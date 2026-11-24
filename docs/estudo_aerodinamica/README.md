# Estudo: aerodinâmica na simulação Gazebo (x500 e alvos)

Estudo de campo e de viabilidade, de 2026-11-05 (capítulos 01–06), seguido da implementação
(capítulo 07, pacote `ros2_ws/src/aero_gz_physics`). Os scripts em `scripts/` são independentes.

| Arquivo | Conteúdo |
|---|---|
| [01_estado_atual_gazebo.md](01_estado_atual_gazebo.md) | O que o Gazebo/PX4 modela hoje (lido na fonte) e o que falta |
| [02_fundamentos_aerodinamica_multirotor.md](02_fundamentos_aerodinamica_multirotor.md) | Efeitos físicos, ordens de grandeza, dados medidos (NASA, ArduPilot, ENAC, UZH, TU Delft), placa plana |
| [03_simuladores_comparados.md](03_simuladores_comparados.md) | Pegasus/Isaac Sim, AirSim, RotorPy, ArduPilot SITL, PX4 SIH, Flightmare, gym-pybullet-drones, MuJoCo, NASA, NeuroBEM |
| [04_modelo_tabelado.md](04_modelo_tabelado.md) | Formulação do modelo pré-calculado (α, β, V), fontes para gerar as tabelas, formato, vento |
| [05_viabilidade_e_desempenho.md](05_viabilidade_e_desempenho.md) | Onde o plugin entra, custo medido, experimento sem código (`Hydrodynamics`), impacto no MPC e no EKF2, veredito |
| [06_plano_implementacao.md](06_plano_implementacao.md) | Plano em fases, estrutura de arquivos, validação e esforço |
| [07_resultados.md](07_resultados.md) | **Implementação (`aero_gz_physics`), validação, ensaios comparativos, custo, GPU e interface** |
| [referencias.md](referencias.md) | Código, documentação e artigos consultados |
| `resultados/` | Dados brutos dos ensaios (`dados/`), figuras e métricas (`figuras/`) |
| `scripts/ordens_de_grandeza.py` (+ `.out.txt`) | Forças e velocidades de regime do x500, com e sem arrasto de corpo |
| `scripts/bench_lookup.cpp` (+ `.out.txt`) | Custo por passo de um modelo tabelado 6-DOF |

## Resumo

1. **O motor de física (DART) não modela o ar.** Toda a aerodinâmica do x500 está no
   `MulticopterMotorModel`: empuxo k_f ω², reação k_m T e um **rotor drag linear** −|ω| c_d v⊥ (com
   vento), mais um pequeno momento de flapping. **Não há arrasto de corpo**, nem força ou momento
   com α e β, nem variação do empuxo com a velocidade. O mundo não carrega `WindEffects`.
2. **Consequência:** só com o arrasto linear, o x500 simulado chega a ~29 m/s com 20° de inclinação
   em regime. Com o arrasto de corpo medido em túnel para drones parecidos (NASA, Iris+/SOLO/Phantom:
   CdA ≈ 0,017–0,042 m²), seriam ~11–15 m/s. Abaixo de ~5 m/s (os cenários atuais, de 2–5 m/s) a
   diferença é pequena: o arrasto de corpo fica em 15–40 % do arrasto de rotor.
3. **Os simuladores de referência fazem pouco aqui:** o Pegasus usa arrasto linear constante e não
   tem vento. RotorPy e ArduPilot usam arrasto quadrático por eixo. O AirSim usa placa plana por face.
   Nenhum usa uma tabela de corpo em (α, β). Tabelas são a prática de JSBSim, NASA e TU Delft.
4. **O modelo tabelado é viável e leve:** ~150 ns por avaliação (≈ 0,04 % de um núcleo para 10
   drones a 250 Hz). Cabe num `gz::sim::System` compilado no nosso pacote, sem recompilar o Gazebo
   nem editar o PX4, e somado ao wrench do `MulticopterMotorModel`.
5. **Placa plana sozinha** é uma boa ordem zero. O melhor uso é **offline**: painéis sobre a geometria
   geram a forma da tabela em (α, β), e a escala é calibrada por um ponto medido (NASA, ou voo real
   pelo método do ArduPilot).
6. **Recomendação:** fazer primeiro a Fase 0, um experimento de 1 dia com o sistema `Hydrodynamics`
   do próprio gz-sim como arrasto quadrático, para medir o impacto nas métricas de perseguição.
   Se for relevante, seguir para a Fase 1: plugin de corpo tabelado + vento Dryden coerente com os
   rotores. Rotor avançado (inflow, VRS, efeito solo) e esteira do alvo ficam como opcionais.
7. **Acoplamento com o código existente:** o `DynamicMpcController` e a planeza usam D = 0,248 N·s/m
   linear. Com arrasto de corpo, esse D precisa virar D_eff(v) configurável. O PX4 EKF2 ganha a
   chance de testar a estimação de vento por arrasto (`EKF2_BCOEF ≈ 69 kg/m²`, `EKF2_MCOEF ≈ 0,12 s⁻¹`).
