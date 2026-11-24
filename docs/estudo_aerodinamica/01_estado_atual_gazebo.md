# 1. O que a simulação atual modela (Gazebo Harmonic + PX4)

Levantamento feito no ambiente instalado em 2026-11-05: `gz sim` 8.15 (Harmonic, via
`ros-jazzy-gz-sim-vendor`), PX4-Autopilot em `~/PX4-Autopilot` (commit `ff5b948436`), mundo `lawn`.

## 1.1 Motor de física

| Item | Valor | Onde |
|---|---|---|
| Sistema de física | `gz::sim::systems::Physics` → plugin padrão `gz-physics-dartsim` | `Tools/simulation/gz/server.config` |
| `<physics type="ode">` | Herança do Gazebo Classic: no gz-sim o motor é escolhido pelo sistema `Physics` (`<engine><filename>` ou `--physics-engine`); sem isso, DART | `worlds/lawn.sdf` |
| Passo | `max_step_size = 0,004 s` (250 Hz), `real_time_factor = 1` | `worlds/lawn.sdf` |
| Gravidade | 9,8 m/s² | `worlds/lawn.sdf` |
| Atmosfera | `<atmosphere type="adiabatic"/>` (só alimenta barômetro e `AirSpeed`; não gera forças) | `worlds/lawn.sdf` |
| Vento | entidade `Wind` padrão do mundo (velocidade 0). **Não há sistema `WindEffects` carregado**, nem no `lawn` nem no `windy` | `server.config`, `worlds/*.sdf` |
| Vento do projeto | *Adendo (fase de implementação):* o `aero_pursuit` já tinha um `WindSystem` próprio (zonas, frentes, rajadas, turbulência), carregado em execução pelo `wind_node`. Ele aplicava só F = c_d·v_vento no corpo, e os rotores não viam esse vento. Com o `aero_gz_physics`, esse vento passa a ser visto pelo corpo e pelos rotores | `aero_pursuit/src/sim/wind_system.cpp` |

O DART integra corpos rígidos, juntas e contatos. **Nenhum motor de física do Gazebo (DART, Bullet,
TPE) modela o ar.** Toda força aerodinâmica vem de *systems* (plugins) que escrevem um wrench
externo nos links a cada passo. Isso é bom para o nosso caso: um modelo aerodinâmico novo é só
mais um *system*, sem recompilar o Gazebo.

## 1.2 Forças aerodinâmicas que existem hoje no x500

O x500 (`x500_base` → `x500` → `x500_depth`) tem apenas 4 instâncias de
`gz::sim::systems::MulticopterMotorModel` (mais o `MotorFailureSystem`). `<velocity_decay/>` está
vazio. Os alvos X3, X4 e Tiltrotor (`ros2_ws/src/aero_pursuit/gz_models/`) usam o mesmo
`MulticopterMotorModel` + `MulticopterControl`. O Tiltrotor do PX4 tem `LiftDrag` nas asas; a cópia
do projeto não.

Modelo implementado em `gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc`
(lido na fonte), por rotor *i* com velocidade ω e eixo **n**:

| Termo | Fórmula | x500 |
|---|---|---|
| Empuxo | T = k_f ω² ao longo de **n** | k_f = 8,54858·10⁻⁶ N/(rad/s)² |
| Torque de reação (yaw) | M_z = −s · k_m · T | k_m = 0,016 m |
| "Rotor drag" (H-force + arrasto induzido lineares) | **F** = −\|ω\| c_d **v**⊥, com **v**⊥ = (**v**_link − **v**_vento) sem a componente em **n** | c_d = 8,06428·10⁻⁵ |
| Momento de rolagem (flapping) | **M** = −\|ω\| c_r **v**⊥ | c_r = 1·10⁻⁶ |
| Dinâmica do motor | 1ª ordem no comando (subida 12,5 ms, descida 25 ms) | — |

O que **não** existe:

- arrasto parasita (quadrático) do corpo, braços, trem de pouso, bateria e câmera;
- sustentação/força lateral e momentos aerodinâmicos do corpo (dependência de α e β);
- variação do empuxo com a velocidade (sustentação translacional, perda de empuxo subindo, razão
  de avanço), efeito solo, vortex ring state na descida;
- amortecimento aerodinâmico de rotação (só o que sai indiretamente do rotor drag aplicado a
  z = +0,06 m);
- vento turbulento/rajadas e esteira (downwash) de outro drone.

Observações da leitura da fonte:

- O vento **é** subtraído da velocidade do link no rotor drag. Ou seja, um `<wind>` no mundo já
  afeta o x500 hoje, mas só pelo termo linear do rotor.
- A força de rotor drag é aplicada no link do rotor (z = +0,06 m acima de `base_link`), o que gera
  um momento de arfagem em voo à frente; isso está certo qualitativamente.
- `rotorVelocitySlowdownSim = 10` só altera a junta visual; as forças usam a velocidade "real".

## 1.3 Consequência numérica (`scripts/ordens_de_grandeza.py`)

Coeficiente linear equivalente no pairar: D = 4 c_d ω_h = **0,248 N·s/m** (ω_h = 769 rad/s), que é o
mesmo valor que o `DynamicMpcController` já usa (`docs/INVESTIGACOES.md` §10). Isso equivale a um
`EKF2_MCOEF` = D/m = 0,12 s⁻¹, próximo do padrão do PX4 (0,15).

Velocidade de regime com altitude mantida, x500 de 2,064 kg:

| Inclinação | Gazebo atual (só rotor drag) | + corpo, CdA = 0,017 / 0,030 / 0,042 m² |
|---|---|---|
| 10° | 14,3 m/s | 9,5 / 8,2 / 7,3 m/s |
| 20° | 28,8 m/s | 15,2 / 12,5 / 11,0 m/s |
| 30° | 43,8 m/s | 19,6 / 15,8 / 13,9 m/s |
| 45° | 68,6 m/s | 26,1 / 20,8 / 18,1 m/s |

(A faixa de CdA vem dos ensaios em túnel de vento da NASA com Iris+, 3DR SOLO e DJI Phantom 3,
airframe sem rotores. Ver [02](02_fundamentos_aerodinamica_multirotor.md).)

O simulado e o real só ficam próximos abaixo de ~5 m/s. Nessa faixa o arrasto do corpo é 15–40 % do
arrasto de rotor (0,5–1,2 N contra 0,07–0,5 N). Acima de ~13,5 m/s o arrasto do corpo domina. Sem
ele, o x500 simulado acelera e freia com folga irreal. Também não "sente" o vento como um drone
real.

Os cenários atuais do projeto têm alvos a 2–5 m/s (`scenarios/*.yaml`). **Hoje a discrepância é
pequena, mas cresce rápido** se os cenários ficarem mais agressivos (perseguição a 8–15 m/s,
vento, rajadas).
