// Benchmark do custo por passo de um modelo aerodinâmico tabelado (estudo de viabilidade).
// Interpolação trilinear em (α, β, V) de 6 coeficientes [CX CY CZ Cl Cm Cn] + montagem do wrench.
// Compilar: g++ -O2 -std=c++17 bench_lookup.cpp -o bench_lookup && ./bench_lookup
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

struct Axis {
  double min, max;
  int n;
  // Índice inferior e fração (grade uniforme → O(1), sem busca binária).
  inline void locate(double x, int& i, double& t) const {
    const double h = (max - min) / (n - 1);
    double u = (std::fmin(std::fmax(x, min), max) - min) / h;
    i = std::min(static_cast<int>(u), n - 2);
    t = u - i;
  }
};

class AeroTable {
 public:
  AeroTable(Axis a, Axis b, Axis v) : a_(a), b_(b), v_(v), data_(size_t(a.n) * b.n * v.n * 6) {
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> d(-1.f, 1.f);
    for (auto& x : data_) x = d(rng);
  }

  inline std::array<double, 6> eval(double alpha, double beta, double speed) const {
    int ia, ib, iv;
    double ta, tb, tv;
    a_.locate(alpha, ia, ta);
    b_.locate(beta, ib, tb);
    v_.locate(speed, iv, tv);
    std::array<double, 6> out{};
    for (int c = 0; c < 8; ++c) {
      const int da = c & 1, db = (c >> 1) & 1, dv = (c >> 2) & 1;
      const double w = (da ? ta : 1 - ta) * (db ? tb : 1 - tb) * (dv ? tv : 1 - tv);
      const float* p = &data_[((size_t(iv + dv) * b_.n + (ib + db)) * a_.n + (ia + da)) * 6];
      for (int k = 0; k < 6; ++k) out[k] += w * p[k];
    }
    return out;
  }

 private:
  Axis a_, b_, v_;
  std::vector<float> data_;
};

}  // namespace

int main() {
  const double pi = std::acos(-1.0);
  AeroTable table({-pi, pi, 73}, {-pi / 2, pi / 2, 37}, {0.0, 25.0, 12});
  std::mt19937 rng(2);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  constexpr int kN = 2'000'000;
  std::vector<std::array<double, 3>> vb(kN);  // velocidade relativa ao ar no corpo
  for (auto& v : vb) v = {15 * u(rng), 15 * u(rng), 5 * u(rng)};

  double sink = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (const auto& v : vb) {
    const double V = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    const double alpha = std::atan2(v[2], v[0]);
    const double beta = V > 1e-6 ? std::asin(v[1] / V) : 0.0;
    const auto c = table.eval(alpha, beta, V);
    const double q = 0.5 * 1.225 * V * V, S = 0.03, l = 0.25;
    sink += q * S * (c[0] + c[1] + c[2]) + q * S * l * (c[3] + c[4] + c[5]);
  }
  const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count() / kN;
  std::printf("custo médio por avaliação: %.1f ns (sink %.3g)\n", ns, sink);
  std::printf("a 250 Hz × 10 veículos: %.4f %% de um núcleo\n", ns * 250 * 10 / 1e9 * 100);
  return 0;
}
