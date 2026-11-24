# Métricas dos ensaios (geradas por tools/plot_results.py)
- Arrasto em α = 0 (painéis): pressão 0.0395 m², atrito 0.00226 m² (5.4 % do total).

## Degrau de 0 → 12 m/s (controlador estilo PX4)

| nível | v em 17,5 s [m/s] | inclinação de cruzeiro [°] | empuxo dos motores [N] | t até 90 % [s] | distância de frenagem [m] |
|---|---|---|---|---|---|
| Gazebo puro | 12.00 | 8.3 | 20.0 | 1.76 | 9.9 |
| linear (Pegasus) | 11.98 | 11.4 | 20.4 | 1.92 | 10.0 |
| quadrático (RotorPy) | 11.94 | 15.7 | 21.3 | 2.16 | 11.2 |
| painéis (AirSim) | 11.94 | 17.7 | 20.5 | 2.16 | 11.3 |
| tabela α×β | 11.95 | 16.8 | 20.0 | 2.04 | 11.0 |
| tabela + BEMT | 11.96 | 16.8 | 19.8 | 2.32 | 10.7 |
| tabela + BEMT + solo + VRS | 11.96 | 16.8 | 19.8 | 2.32 | 10.3 |

## Pairando com vento de 6 m/s + rajada 1−cos de +6 m/s

| nível | inclinação com vento [°] | desvio de vx na rajada [m/s] | deriva final [m] |
|---|---|---|---|
| Gazebo puro | 4.31 | -0.389 | -2.30 |
| linear (Pegasus) | 5.84 | -0.518 | -3.06 |
| quadrático (RotorPy) | 6.06 | -0.781 | -3.29 |
| painéis (AirSim) | 6.70 | -0.785 | -3.40 |
| tabela α×β | 6.28 | -0.735 | -3.23 |
| tabela + BEMT | 6.12 | -0.669 | -2.85 |
| tabela + BEMT + solo + VRS | 6.12 | -0.669 | -2.85 |

## Descida vertical a −7 m/s (≈ 1,1 v_h)

| nível | vz médio [m/s] | σ(vz) [m/s] | empuxo dos motores médio [N] | σ(empuxo) [N] | σ(roll) [°] |
|---|---|---|---|---|---|
| tabela α×β | -6.96 | 0.028 | 17.80 | 0.01 | 0.00 |
| tabela + BEMT | -6.76 | 0.165 | 13.20 | 0.03 | 0.00 |
| tabela + BEMT + VRS | -7.07 | 0.145 | 22.21 | 0.88 | 2.40 |

## Drone pairando a 4 m com outro passando 2,5 m acima a 2 m/s

| variante | queda máxima de altitude [m] | empuxo máximo dos motores [N] | |pitch| máx. [°] |
|---|---|---|---|
| sem esteira | 0.000 | 20.24 | 0.00 |
| com esteira | 0.375 | 26.64 | 3.27 |

## Custo por passo de simulação (gz-sim headless, 250 Hz)

| drones | Gazebo puro | linear (Pegasus) | quadrático (RotorPy) | painéis (AirSim) | tabela α×β | tabela + BEMT | tabela + BEMT + solo + VRS |
|---|---|---|---|---|---|---|---|
| 1 | 154 µs | 175 µs | 175 µs | 177 µs | 175 µs | 176 µs | 180 µs |
| 4 | 440 µs | 479 µs | 478 µs | 487 µs | 491 µs | 493 µs | 493 µs |
| 8 | 772 µs | 939 µs | 893 µs | 901 µs | 936 µs | 924 µs | 928 µs |
| 16 | 1467 µs | 1656 µs | 1653 µs | 1696 µs | 1660 µs | 1757 µs | 1727 µs |

## Núcleo (ns por avaliação)

| componente | ns |
|---|---|
| corpo linear | 5.0 |
| corpo quadrático | 8.4 |
| corpo tabela α×β | 109.0 |
| corpo painéis (126) | 987.0 |
| corpo painéis (1134) | 8788.1 |
| rotor efeito solo | 45.6 |
| rotor BEMT | 178.7 |
| rotor BEMT+solo+VRS | 234.8 |
| esteira (1 ponto) | 25.1 |

## GPU × CPU (painéis em tempo real, µs por passo)

| painéis | drones | GPU | CPU |
|---|---|---|---|
| 126 | 1 | 264 | 0.7 |
| 126 | 4 | 278 | 2.6 |
| 126 | 16 | 276 | 12.4 |
| 126 | 64 | 272 | 63.2 |
| 126 | 256 | 279 | 267.5 |
| 126 | 1024 | 335 | 1090.8 |
| 2016 | 1 | 340 | 17.5 |
| 2016 | 4 | 442 | 190.9 |
| 2016 | 16 | 475 | 816.8 |
| 2016 | 64 | 405 | 1708.7 |
| 2016 | 256 | 500 | 6761.9 |
| 2016 | 1024 | 758 | 23177.3 |
