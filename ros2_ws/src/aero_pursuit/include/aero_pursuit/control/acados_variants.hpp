#pragma once

#include "aero_pursuit/control/acados_api.hpp"

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace aero_pursuit::control
{

/// Solvers for other (N, dt) pairs, generated and compiled at run time and kept for a fast switch.
///
///   request(N, dt): already loaded -> returned at once; compiled before (disk cache) -> loaded
///                   (dlopen, a few ms); otherwise starts generate.py --N --dt in a background
///                   thread (~3 s) and returns nothing until poll() finds it ready.
///
/// Cache: $XDG_CACHE_HOME/aero_pursuit/acados/<name>_<hash>/ (hash of pursuit_ocp.py + acados
/// version: editing the problem invalidates the old builds). Needs the acados venv and a compiler
/// on this computer (not on the Raspberry Pi runtime image: request() then reports the failure).
class AcadosVariants
{
public:
  AcadosVariants();
  ~AcadosVariants();
  AcadosVariants(const AcadosVariants &) = delete;
  AcadosVariants & operator=(const AcadosVariants &) = delete;

  /// Variant (N, dt) if it can be used now; empty while it is being built (or if it failed).
  const AcadosApi * request(int N, double dt);
  /// Finishes a background build: true when the last requested variant became available.
  bool poll();
  /// A build is running right now (a finished build waiting for poll() does not count).
  bool building() const
  {
    return build_.valid() && build_.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
  }
  /// Name of the variant being built (e.g. "pursuit_N30_dt100"), empty if none.
  const std::string & buildingName() const {return building_;}
  /// Last event worth logging ("compiling ...", "ready ...", "failed ..."), then cleared.
  std::string takeMessage();

  static std::string variantName(int N, double dt);

private:
  struct Library;  ///< dlopen handle + its AcadosApi (RAII: dlclose)

  std::string libraryPath(const std::string & name) const;
  const AcadosApi * load(const std::string & name, int N, double dt);

  std::string cache_dir_;
  std::map<std::string, std::unique_ptr<Library>> loaded_;
  std::future<bool> build_;            ///< generate.py running for `building_`
  std::string building_;
  int building_N_ = 0;
  double building_dt_ = 0.0;
  std::string message_;
};

}  // namespace aero_pursuit::control
