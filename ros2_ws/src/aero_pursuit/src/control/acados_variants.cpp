#include "aero_pursuit/control/acados_variants.hpp"

#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>

// Set by CMake: acados prefix (venv/bin/python), installed acados/generate.py and the hash of
// pursuit_ocp.py + acados version (the same one that decides the build-time regeneration).
#ifndef AERO_ACADOS_ROOT
#define AERO_ACADOS_ROOT ""
#endif
#ifndef AERO_ACADOS_GENERATOR
#define AERO_ACADOS_GENERATOR ""
#endif
#ifndef AERO_ACADOS_HASH
#define AERO_ACADOS_HASH "nohash"
#endif

namespace aero_pursuit::control
{

namespace fs = std::filesystem;

namespace
{
std::string quote(const std::string & s)
{
  std::string out = "'";
  for (char c : s) {
    out += c == '\'' ? std::string("'\\''") : std::string(1, c);
  }
  return out + "'";
}

std::set<std::string> & failedVariants()
{
  static std::set<std::string> failed;  // built once and failed: not retried in this process
  return failed;
}
}  // namespace

struct AcadosVariants::Library
{
  void * handle = nullptr;
  AcadosApi api;

  ~Library()
  {
    if (handle) {
      dlclose(handle);
    }
  }
};

AcadosVariants::AcadosVariants()
{
  const char * xdg = std::getenv("XDG_CACHE_HOME");
  const char * home = std::getenv("HOME");
  const fs::path base = xdg && *xdg ? fs::path(xdg) : fs::path(home ? home : "/tmp") / ".cache";
  cache_dir_ = (base / "aero_pursuit" / "acados").string();
}

AcadosVariants::~AcadosVariants()
{
  if (build_.valid()) {
    build_.wait();  // a compiler still writing into the cache
  }
}

std::string AcadosVariants::variantName(int N, double dt)
{
  // Same rule as pursuit_ocp.variant_name: pursuit_mpc_N<N>_dt<milliseconds>.
  std::ostringstream s;
  s << "pursuit_mpc_N" << N << "_dt" << std::lround(dt * 1000.0);
  return s.str();
}

std::string AcadosVariants::libraryPath(const std::string & name) const
{
  const std::string hash = std::string(AERO_ACADOS_HASH).substr(0, 12);
  return (fs::path(cache_dir_) / (name + "_" + hash) / name /
         ("libacados_ocp_solver_" + name + ".so")).string();
}

const AcadosApi * AcadosVariants::load(const std::string & name, int N, double dt)
{
  // Idempotent: a variant is loaded once. Loading it again would replace (and dlclose) the
  // library a running solver still calls into.
  if (const auto it = loaded_.find(name); it != loaded_.end()) {
    return &it->second->api;
  }
  const auto path = libraryPath(name);
  auto lib = std::make_unique<Library>();
  // RTLD_LOCAL: each variant keeps its symbols to itself; we only reach them through dlsym.
  lib->handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!lib->handle) {
    message_ = "could not load " + path + ": " + dlerror();
    return nullptr;
  }
  auto sym = [&](const char * suffix) {return dlsym(lib->handle, (name + suffix).c_str());};
  AcadosApi & a = lib->api;
  a.name = name;
  a.N = N;
  a.dt = dt;
  // dlsym returns void *; the functions really take a <name>_solver_capsule *, which has the
  // same representation as void * (an opaque pointer), so calling through these types is safe.
  a.create_capsule = reinterpret_cast<void * (*)()>(sym("_acados_create_capsule"));
  a.create = reinterpret_cast<int (*)(void *)>(sym("_acados_create"));
  a.solve = reinterpret_cast<int (*)(void *)>(sym("_acados_solve"));
  a.free_solver = reinterpret_cast<int (*)(void *)>(sym("_acados_free"));
  a.free_capsule = reinterpret_cast<int (*)(void *)>(sym("_acados_free_capsule"));
  a.get_config = reinterpret_cast<void * (*)(void *)>(sym("_acados_get_nlp_config"));
  a.get_dims = reinterpret_cast<void * (*)(void *)>(sym("_acados_get_nlp_dims"));
  a.get_in = reinterpret_cast<void * (*)(void *)>(sym("_acados_get_nlp_in"));
  a.get_out = reinterpret_cast<void * (*)(void *)>(sym("_acados_get_nlp_out"));
  a.update_params = reinterpret_cast<int (*)(void *, int, double *, int)>(
    sym("_acados_update_params"));
  if (!a.create_capsule || !a.create || !a.solve || !a.free_solver || !a.free_capsule ||
    !a.get_config || !a.get_dims || !a.get_in || !a.get_out || !a.update_params)
  {
    message_ = "library " + path + " without the functions " + name + "_acados_*";
    return nullptr;
  }
  const AcadosApi * api = &lib->api;
  loaded_[name] = std::move(lib);
  return api;
}

const AcadosApi * AcadosVariants::request(int N, double dt)
{
  const auto name = variantName(N, dt);
  if (const auto it = loaded_.find(name); it != loaded_.end()) {
    return &it->second->api;                       // in memory: instant
  }
  if (name == building_) {
    return nullptr;  // still being built: the .so may already exist but be incomplete
  }
  if (fs::exists(libraryPath(name))) {
    return load(name, N, dt);                      // compiled before (disk cache): dlopen
  }
  if (failedVariants().count(name) || build_.valid()) {
    return nullptr;                                // failed before / another build running
  }
  const std::string root = AERO_ACADOS_ROOT;
  const std::string generator = AERO_ACADOS_GENERATOR;
  const fs::path python = fs::path(root) / "venv" / "bin" / "python";
  if (root.empty() || !fs::exists(python) || !fs::exists(generator)) {
    failedVariants().insert(name);
    message_ = "cannot generate " + name + " here (no acados venv in " + root +
      " or no " + generator + ")";
    return nullptr;
  }
  const fs::path dir = fs::path(libraryPath(name)).parent_path().parent_path();
  fs::create_directories(dir.parent_path());
  std::ostringstream cmd;
  cmd << "ACADOS_SOURCE_DIR=" << quote(root) << ' ' << quote(python.string()) << " -W ignore "
      << quote(generator) << ' ' << quote(dir.string()) << " --N " << N << " --dt " << dt
      << " > " << quote(dir.string() + ".log") << " 2>&1";
  building_ = name;
  building_N_ = N;
  building_dt_ = dt;
  build_ = std::async(std::launch::async, [c = cmd.str()]() {return std::system(c.c_str()) == 0;});
  message_ = "compiling " + name + " (log: " + dir.string() + ".log)";
  return nullptr;
}

bool AcadosVariants::poll()
{
  if (!build_.valid() || build_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return false;
  }
  const bool ok = build_.get();  // invalidates the future
  const auto name = building_;
  building_.clear();
  if (!ok) {
    failedVariants().insert(name);
    message_ = "generation failed for " + name + " (log: " +
      fs::path(libraryPath(name)).parent_path().parent_path().string() + ".log)";
    return false;
  }
  if (!load(name, building_N_, building_dt_)) {  // load() wrote the reason in message_
    failedVariants().insert(name);
    return false;
  }
  message_ = "ready: " + name;
  return true;
}

std::string AcadosVariants::takeMessage()
{
  std::string m;
  m.swap(message_);
  return m;
}

}  // namespace aero_pursuit::control
