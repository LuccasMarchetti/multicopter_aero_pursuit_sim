# Investigações técnicas

Registro das análises feitas na expansão do `aero_pursuit` (controle, simulação, telemetria e
infraestrutura), com o que foi verificado, medido e decidido. Valores medidos nesta máquina:
Radeon RX 6600 (Mesa 25.2, RADV/radeonsi), ROS 2 Jazzy, Gazebo Harmonic 8.

---

## 1. Parâmetros físicos do `x500_depth`

O modelo é composto por herança de SDFs no PX4-Autopilot
(`~/PX4-Autopilot/Tools/simulation/gz/models/`):

| Arquivo | O que define |
|---|---|
| `x500_depth/model.sdf` | Inclui `x500` + `OakD-Lite` (câmera em `base_link` + [0,12 0,03 0,242] m) e a junta fixa `CameraJoint` |
| `x500/model.sdf` | Inclui `x500_base` e os 4 plugins `gz::sim::systems::MulticopterMotorModel` (constantes dos motores) |
| `x500_base/model.sdf` | Massa e inércia de `base_link` e dos rotores, geometria e posições dos rotores |
| `OakD-Lite/model.sdf` | Sensores `IMX214` (RGB 1920×1080, FOV 1,204 rad, 30 Hz) e `StereoOV7251` (profundidade 640×480 `R_FLOAT32`, 0,2–19,1 m, 30 Hz) |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` (e `4002_gz_x500_depth`) | Alocação de controle (`CA_ROTOR*_PX/PY/KM`), `MPC_THR_HOVER = 0,6`, mapeamento dos ESCs simulados |
| `src/modules/simulation/gz_bridge/server.config` | Sistemas do servidor (inclui `Sensors` com `render_engine ogre2`) |

| Grandeza | Valor | Origem |
|---|---|---|
| Massa de `base_link` | 2,0 kg | `x500_base` |
| Massa de cada rotor | 0,01608 kg | `x500_base` |
| Massa total | **2,064 kg** | soma |
| Inércia de `base_link` (Ixx, Iyy, Izz) | 0,02167; 0,02167; 0,04 kg·m² | `x500_base` |
| Posição dos rotores | (±0,174; ±0,174; 0,06) m → braço 0,246 m | `x500_base` |
| Constante de empuxo `motorConstant` (kf) | 8,54858·10⁻⁶ N/(rad/s)² | `x500` |
| Constante de torque `momentConstant` (km) | 0,016 m | `x500` |
| Velocidade máxima do rotor | 1000 rad/s | `x500` |
| Constantes de tempo do motor (subida/descida) | 0,0125 s / 0,025 s | `x500` |
| Arrasto do rotor / momento de rolagem | 8,06428·10⁻⁵ / 1·10⁻⁶ | `x500` |
| `rotorVelocitySlowdownSim` | 10 (só visual da junta) | `x500` |

Grandezas derivadas (implementadas em `control/vehicle_params.hpp`, verificadas por teste):
empuxo máximo 4·kf·ω²max = **34,19 N**; razão empuxo/peso **1,69**; empuxo de pairar **59 %** do
máximo (consistente com `MPC_THR_HOVER = 0,6`); aceleração horizontal máxima mantendo altitude
g·√((T/W)²−1) ≈ **13,3 m/s²**.

Uso no código: a planeza diferencial (`control/flatness.hpp`) converte o comando de aceleração e
jerk em empuxo, atitude e taxas angulares do x500; `/pursuit/metrics` publica `thrust_ratio` e `tilt`
exigidos, mostrando quando um comando está no limite dinâmico do veículo.

## 2. MPC: de ganho fixo para LTV-MPC (QP online)

**Antes:** MPC condensado sobre o duplo integrador de erro com modelo invariante e grade uniforme;
sem restrições, `U* = −H⁻¹F x₀` com ganho pré-computado (equivalente a um LQR de horizonte finito).

**Agora (`mpc.online: true`):** a cada ciclo o QP é montado e resolvido:

- **Modelo variante no tempo:** grade não uniforme `dt₀ = dt real do ciclo`, `dt_k = dt·g^k`
  (`horizon_growth`, padrão 1,15). Com N = 10 o horizonte cobre ~1,0 s em vez de 0,5 s, com
  resolução fina perto do presente. `A_k, B_k` mudam por estágio e o modelo é remontado quando o
  período real do timer varia.
- **Referência variante no tempo:** o ponto de standoff de cada estágio fica sobre a **linha de
  visada prevista** (alvo extrapolado × trajetória anterior do perseguidor, reexpressa no instante
  atual), em vez de congelar a direção da LOS. Isso antecipa a rotação da LOS em manobras laterais.
- **Restrições (hoje no acados, `acados/aero_ocps.py`):** `|a_k| ≤ a_max` dura e `|v_k| ≤ v_max` branda. Na época: projeção na bola (FISTA) e **Lagrangiano
  aumentado** (penalidade ρ + multiplicadores atualizados 4× por ciclo e transportados entre ciclos).
  A penalidade quadrática simples deixava violações de ~35 % (medido: 2,05 m/s para limite 1,5 m/s);
  com o ALM, 1,50 m/s.
- **Custos medidos** (N = 10, nesta máquina): ~0,3–0,45 ms por ciclo com as duas restrições (orçamento: 50 ms).
- `mpc.online: false` mantém o MPC de ganho fixo (e o teste confirma que o LTV-MPC com grade
  uniforme e LOS congelada reproduz exatamente o MPC fixo).

## 3. LQI

Estado aumentado `z = [e_p; e_v; ξ]`, `ξ(k+1) = ξ(k) + dt·e_p(k)`, DARE 9×9 resolvida offline.
Anti-windup por zona (`integral_zone`) e saturação da norma de ξ. Teste com perturbação constante de
0,4 m/s na direção da LOS: o LQR fica com o erro analítico `K_p⁻¹K_v·b ≈ 0,27 m`; o LQI converge para
< 1 cm. Com perturbação 3D, o erro de standoff é só radial e a LOS gira; o LQI reduz o erro pela
metade ou mais em 60 s.

## 4. MIGHTY, splines de Hermite e controle por planeza diferencial

**MIGHTY** (Hermite Spline-Based Efficient Trajectory Planning) otimiza simultaneamente posições,
derivadas nodais e durações de uma spline de Hermite quíntica (d = 5, ν = 2), com custo de tempo,
suavidade (∫|j|² em forma fechada via Bézier) e penalidades suaves de viabilidade (v, a, j) e de
corredores seguros, resolvido por L-BFGS. Propriedades relevantes para perseguição: continuidade C²
por construção, **controle local das derivadas** (velocidade/aceleração em cada nó são variáveis
explícitas) e otimização espaço-temporal conjunta.

**Implementado (`controller_type: hermite`):** replanejamento em horizonte deslizante com um
segmento quíntico de Hermite:

- nó 0 = estado atual (posição, velocidade de referência e aceleração do plano anterior → setpoint
  C² entre ciclos); nó 1 = ponto de standoff previsto em T com velocidade do alvo;
- custo MIGHTY reduzido: `w_T·T + w_j·∫|j|² + w_lim·Σ[(|v|²−v²max)₊² + (|a|²−a²max)₊²]`, com T
  otimizado por seção áurea (único grau de liberdade livre de um segmento; ~28 µs por ciclo);
- comando = avaliação do plano em `t = dt`: velocidade, **aceleração e jerk como feedforward**
  (campo `jerk` do `TrajectorySetpoint`, antes sempre NaN);
- utilitários reutilizáveis: `QuinticHermite` (avaliação, ∫|j|² fechado, mapa Hermite→Bézier) e
  `flatOutputsToState` (Mellinger & Kumar, 2011: aceleração/jerk/yaw → empuxo, atitude, p, q, r).

**Propostas de continuação** (em ordem de retorno esperado):

1. *Multi-segmento à MIGHTY:* 2–3 segmentos com velocidades/acelerações dos nós internos e durações
   como variáveis (L-BFGS), permitindo trajetórias de interceptação que contornem obstáculos ou
   respeitem um cone de visibilidade da câmera (custo de manter o alvo no FOV).
2. *Controle por planeza na malha de atitude:* enviar ao PX4 `VehicleAttitudeSetpoint` /
   `VehicleRatesSetpoint` calculados por `flatOutputsToState` (offboard de atitude), contornando a
   dinâmica do controlador de velocidade do PX4 em manobras agressivas.
3. *Restrições de percepção no plano:* penalizar o ângulo entre o eixo óptico (dependente de
   atitude, recuperável pela planeza) e a LOS ao longo do plano — "perception-aware planning".

## 5. Outras técnicas avançadas para perseguição

| Técnica | Situação | Motivo |
|---|---|---|
| **ASMC** (modos deslizantes adaptativo) | Implementado (`asmc`) | Robusto à aceleração desconhecida do evasor sem conhecer seu limitante; camada-limite e σ-modificação evitam *chattering* e deriva do ganho |
| **LQI** | Implementado | Erro de regime nulo |
| **LTV-MPC com restrições** | Implementado | Ver seção 2 |
| **NMPC** (acados / CasADi) | Proposto | Modelo não linear com a dinâmica da malha de atitude do PX4 (1ª ordem, τ ≈ 0,1–0,2 s) e custos não lineares diretos em distância e ângulo da LOS (ou em coordenadas de imagem, IBVS-MPC). Exige dependência externa (acados) e código gerado; o LTV-MPC com LOS prevista captura boa parte do benefício com QP convexo |
| **IBVS-MPC** | Proposto | Ver artigo "Image-based visual servoing for air-to-air drone tracking with MPC" em `source/`; útil quando a profundidade some (substituiria a aproximação por LOS do modo APPROACH) |
| **Tube MPC / MPC robusto** | Proposto | Garante restrições sob o erro do KF; o tubo poderia ser dimensionado pela covariância do estimador |
| **Jogos diferenciais / RL multiagente** | Infraestrutura pronta | O ambiente `aero_pursuit_rl` suporta self-play perseguidor × evasor (seção 11 do README) |

## 6. Aceleração em hardware no Gazebo

- O `server.config` do PX4 já carrega `gz::sim::systems::Sensors` com `render_engine ogre2`, e o
  OGRE2 usa a GPU sempre que o driver OpenGL é de hardware.
- **Verificação objetiva:** `/proc/<pid>/fdinfo` do processo do Gazebo mostra `drm-driver: amdgpu`
  e tempo crescente em `drm-engine-gfx` (mundo de teste 640×480 @ 30 Hz: ~28 fps entregues, ~49 ms de
  GPU acumulados nos primeiros segundos). O mesmo vale em modo headless com `--headless-rendering`
  (EGL), que agora é usado quando `GZ_HEADLESS=1` ou em sessões SSH (antes o `-s` sem EGL podia cair
  em renderização por software). Diagnóstico: `ros2_ws/check_gpu_rendering.sh`.
- **Vulkan:** `--render-engine-api-backend vulkan` aborta nesta instalação
  (`RenderSystem_Vulkan.so: undefined symbol glslang::InitializeProcess` no `gz_ogre_next_vendor`).
  O launcher detecta símbolos não resolvidos com `ldd -r` e volta para OpenGL.
- **Física:** o ODE/DART do gz-sim roda na CPU (não há física em GPU no Gazebo Harmonic); o passo é
  0,004 s a 250 Hz (`lawn.sdf`).
- **Gargalo restante na CPU:** a conversão/cópia das imagens no `ros_gz_bridge` (RGB 1920×1080 =
  ~186 MB/s a 30 Hz). Mitigações já presentes: ponte `lazy`, filas de 1 quadro e intra-processo com a
  visão. Reduzir a resolução exigiria um modelo derivado do OakD-Lite (fora da árvore do PX4).

## 7. Ponte de TF: `gazebo_tf_bridge_node` é necessário?

Não. Foi testado em um Gazebo headless isolado:

1. O sistema `gz::sim::systems::PosePublisher` pode ser anexado **em tempo de execução** a modelos já
   inseridos pelo serviço `/world/<w>/entity/system/add` (`gz.msgs.EntityPlugin_V`, entidade por id).
2. Ele publica `gz.msgs.Pose_V` em `/world/<w>/model/<modelo>/pose` com o cabeçalho
   `frame_id = <mundo>`, `child_frame_id = <modelo>` e carimbo do tempo de simulação.
3. O `ros_gz_bridge` converte `Pose_V → tf2_msgs/TFMessage` nativamente, preservando os dois frames.

Resultado: `/tf` com `lawn → X4` etc. sem nó tradutor (`config/gz_bridge_lawn_tf.yaml` +
`gz_pose_tf.sh`, chamado pelo `launch_gz_and_drones.sh`, idempotente e à espera do `x500_depth_0`,
que só surge com o PX4). Diferenças em relação ao nó: a taxa é a `update_frequency` do plugin
(50 Hz) e só os modelos listados publicam (sem o filtro sobre o `dynamic_pose/info` de todos os
links). O nó continua disponível com `tf_source:=node`. Adicionar o plugin direto nos SDFs exigiria
alterar o modelo do PX4 (fora do repositório); a abordagem por serviço evita isso.

## 8. Logs e o travamento no encerramento

- Os nós do pacote só emitem logs de eventos (sem mensagens por ciclo); o ruído vinha do INFO do
  `ros_gz_bridge` (uma linha por ponte), do Foxglove Bridge (conexões/assinaturas), do MicroXRCEAgent
  (nível 4) e do console do PX4. Padrões agora: WARN nos nós ROS, `-v 3` no agente, `-v 1` no
  Gazebo, `PX4_QUIET=1` opcional; `sim_manager.sh` separa um arquivo por processo.
- **Achado:** o container **segfaultava ao encerrar** (sinal 11; o apport gerando o core parecia um
  travamento e o launch escalava até SIGKILL). Bissecção carregando um componente por vez: só o
  `ros_gz_bridge::RosGzBridge` falha (todos os nós do `aero_pursuit` encerram com código 0), em uma
  thread executando código de uma biblioteca já descarregada → o `dlclose` da `libros_gz_bridge.so`
  ocorre com threads do gz-transport vivas. Com `LD_PRELOAD` da biblioteca (que nunca é
  descarregada) o container sai com código 0 e o rosbag é finalizado (`metadata.yaml` +
  `session_info.yaml`) — antes, o bag da sessão corrente podia ficar sem índice.

## 9. Joystick

O mapeamento de eixos e sinais do `JoyTeleopNode` era idêntico ao do último commit do autor antes
da refatoração (5be8cd9: x ← −eixo 3, y ← −eixo 2, z ← −eixo 1, yaw ← −eixo 0). O que mudou no
commit 90ae9b6 foi o launch passar a iniciar o `joy_node`, cujo layout de eixos é o cru do dispositivo
(num controle Xbox/PS4: 2 = gatilho LT/L2, 3 = analógico direito horizontal, 4 = vertical). O mapeamento
original segue o layout SDL do `game_controller_node` (0 LX, 1 LY, 2 RX, 3 RY), que passou a ser o
padrão (`joy_driver`), e os sinais agora são parâmetros (`invert_*`).

**Atualização:** a pedido, todos os sinais foram invertidos (`invert_*: false`, comando = +eixo). O
nó agora também comanda o perseguidor: **LB/RB** alternam alvo ↔ perseguidor (o veículo anterior
recebe comando nulo, pois o `MulticopterVelocityControl` mantém o último Twist; ao assumir o
perseguidor, `~/enable` do `pursuit_controller_node` é desligado e religado na volta). O perseguidor
recebe `TrajectorySetpoint` de velocidade (referencial de rumo → NED pelo `heading` do PX4) no mesmo
tópico do controlador, repassado pelo `offboard_control`. **LT/RT** reduzem/aumentam a escala da
velocidade linear (proporcional à pressão, `speed_scale_rate`/s, entre `speed_scale_min/max`). Veículo
ativo e escala: `/drone_joy_teleop/status`. Com `joy_driver:=joy_node`: LB 4, RB 5, LT 2, RT 5 e
`trigger_released_value: 1.0` (o nó ignora o 0 inicial do `joy_node` antes do primeiro movimento).

## 10. MPC dinâmico (`mpc.mode: dynamic`)

O MPC cinemático usa aceleração como entrada e deixa a dinâmica de atitude para o PX4. O modo
dinâmico (`DynamicMpcController`) prediz a física do x500, linearizada no pairar e no referencial de
rumo (frente/esquerda desacoplados, yaw da `YawPolicy`):

| Eixo | Estado | Entrada | Dinâmica |
|---|---|---|---|
| frente / esquerda | p, v, θ, ω | τ / τ_disp | ṗ = v; m v̇ = m g θ − D v; θ̇ = ω; I ω̇ = τ |
| vertical | p, v | a_z | v̇ = a_z = (T cos φ cos θ − m g)/m |

- **Do SDF:** D = 4 · c_d · ω_hover ≈ 0,248 N·s/m (arrasto `-|ω| c_d v⊥` do `MulticopterMotorModel`);
  τ_max = 4 · 0,174 · Δ ≈ 2,43 N·m com Δ = min(T_max,r − T_h,r, T_h,r) = 3,49 N (`torque_authority`
  reserva parte ao laço interno); Ixx = Iyy = 0,0217 kg·m². O termo giroscópico ω × (Iω) é de 2ª
  ordem no pairar; entra no feedforward de torque (`AttitudeCommand::torque`).
- **Restrições:** |τ| (caixa exata), |θ| ≤ min(`max_tilt_deg`, atan(max_acc/g)) e |v| ≤ max_vel
  (suaves), a_z ∈ [T_min/m − g, T_max cos θ_max/m − g] ∩ ±max_acc. A inclinação combinada dos dois
  eixos é limitada em norma no comando final.
- **Solver (hoje: acados, um OCP acoplado nos três eixos, SQP + HPIPM).** Na época: QP condensado por eixo com **Newton projetado** (Armijo). O modelo de 4ª ordem é mal
  condicionado (posição ← torque passa por três integrações); o FISTA do MPC cinemático precisaria de
  centenas de iterações. Custo medido: ~140 µs por ciclo (N = 20, horizonte de 1 s).
- **Referência:** standoff na LOS prevista e θ_ref = D v_alvo/(m g), a inclinação que equilibra o arrasto.
- **Saída (`mpc.dynamic.output: attitude`):** `VehicleAttitudeSetpoint` em
  `/fmu/in/vehicle_attitude_setpoint_v1` (via `offboard_control`, modo Offboard de atitude): atitude
  prevista `attitude_lead` estágios à frente + yaw da política, e empuxo T = m(g + a_z)/cos(inclinação).
  **Empuxo normalizado:** com `THR_MDL_FAC = 0` o PX4 é linear em ω entre `SIM_GZ_EC_MIN = 150` e
  `SIM_GZ_EC_MAX = 1000` rad/s, então u = (√(T/(4 kf)) − 150)/850, ≈ 0,73 no pairar, e não T/T_max
  (0,59). Com `output: trajectory`, velocidade/aceleração previstas vão em `TrajectorySetpoint`.
- **Estado inicial:** `/fmu/out/vehicle_odometry` (atitude, taxas FRD→FLU, velocidade NED→ENU); sem
  odometria recente, usa a própria predição de 1 passo (laço aberto em atitude).
- **Segurança:** o `offboard_control` volta a HOLD após `attitude_setpoint_timeout = 0,25 s` sem
  setpoints de atitude (inclinação velha é pior que velocidade velha).

## 11. Auditoria de estabilidade

| Ponto | Problema | Correção |
|---|---|---|
| `sim_manager.sh stop` | Só o líder do grupo era verificado; se o `bash -c` saía antes dos filhos (gz, px4), SIGTERM/SIGKILL nunca eram enviados → órfãos | Vivacidade do grupo (`kill -0 -- -pgid`) em `stop`/`status` |
| `launch_gz_and_drones.sh` | `cleanup` não esperava `gz sim`/`px4` nem encerrava `gz_pose_tf.sh` | Encerra os jobs, espera até 10 s e força com SIGKILL; traps INT/TERM |
| `target_estimator` | Medida da câmera (atrasada) rotacionada pela atitude mais recente: erro ≈ ω·atraso·distância | Histórico de atitude (1 s) com slerp no carimbo da medida |
| `target_estimator` | Após reset do Gazebo, `now − last_enu_tf` negativo bloqueava a TF base → enu | Histórico e limitador reiniciados em salto para trás |
| `pursuit_controller` | Carimbos "do futuro" após reset → idade 0 → estimativa velha tratada como recente | Descarta estimativas com carimbo > now + 0,5 s |
| `session_recorder` | Pre-roll nunca expirava com o tempo voltando (crescimento sem limite) | Limpa no salto para trás + teto de 20 000 mensagens |
| `training_arena` | Velocidade do alvo/perseguidor congelada após reset (stamp < anterior) | Reinicia a estimativa |
| `offboard_control`, `joy_teleop` | Requisições de serviço sem resposta acumulavam no cliente a cada nova tentativa | `prune_pending_requests()` antes de reenviar |

Verificados sem alteração: callbacks de cada nó ficam no grupo padrão (mutuamente exclusivo) mesmo no
`component_container_mt`, então não há corrida entre `onRgb`/`onClick`/profundidade/parâmetros; o
quadro BGR é uma cópia própria (não aponta para o buffer da mensagem); a profundidade é mantida por
`shared_ptr<const Image>` (intra-processo sem cópia); o `gz::transport::Node` é destruído antes do
broadcaster que seu callback usa; o `tf2_ros::Buffer` limpa o cache em saltos para trás.

## 12. Alcance da câmera de profundidade, ROI automática e câmera no alvo

Medido em 2026-11-05 com `scenarios/depth_range_probe.yaml`: perseguidor parado a 6 m, partindo de
costas (rumo 180°), X4 se aproximando de 34 m a 8 m a 0,7 m/s. Sonda:
`ros2 run aero_pursuit probe_acquisition.py --out r.json --frames 12` (grava detecção, verdade, rumo,
estatísticas brutas da profundidade e quadros anotados).

**Profundidade (OakD-Lite StereoOV7251, clip 0,2–19,1 m no SDF do PX4).**

| distância [m] | quadros com profundidade | ROI "além do alcance" | erro da profundidade |
|---|---|---|---|
| 8–18 | 90–100 % | 0 % | −0,20 a −0,25 m (face frontal, esperado) |
| 18–20 | 70 % | 30 % | −0,24 m |
| ≥ 20 | 0 % | 97–100 % | — |

Além do far clip o Gazebo devolve **+inf** (nenhum pixel saturado em 19,1). Parâmetros do rastreador:
`depth_min_range` (0,2), `depth_max_range` (19,1), `depth_beyond_fraction` (0,85). Com a ROI além do
alcance, `TargetBearing.range_lower_bound = 19,1 m` e `TargetDetection.depth_beyond_range = true`; o
APPROACH usa `max(estimativa, limite)`.

**Rumo no estágio.** O PX4 controla o rumo estimado pelo EKF, que no mundo `lawn` difere do verdadeiro
em ~7° (o campo magnético do SDF não bate com o WMM que o EKF usa). Um integrador lento (0,6 s⁻¹) sobre o
erro verdadeiro no `scenario_runner` remove o viés: erro de 21,7° no início do estágio para 0,1° no fim;
0,18° em média durante a execução (`face_target`).

**ROI automática.** A esfera de raio fixo virou a projeção dos 8 cantos da caixa visível orientada
(`model_boxes`), com a TF câmera ← alvo no carimbo do quadro (`use_image_stamp`). A primeira caixa do X4
(com o disco das hélices, centro na origem) ficou baixa e larga: o deslocamento × distância foi constante
(−130 px·m, erro métrico e não angular), e a segmentação do drone nos quadros mostrou que o centro
visível fica 0,11 m acima da origem do modelo, com ~0,60 × 0,20 m visíveis. A caixa foi recalibrada para
a geometria visível.

Resultado no mesmo cenário (dados e quadros em `docs/resultados_roi_profundidade/`):

| | antes (caixa com o disco das hélices) | depois (geometria visível, TF no quadro) |
|---|---|---|
| IoU rastreador × projeção | 0,36 | 0,73 |
| erro de centro mediano / p90 | 8,0 / 17,2 px | 3,2 / 5,9 px |
| viés vertical (dy·distância) | −135 px·m | −14 px·m |
| erro de rumo na execução | 0,18° | 0,09° |
| aquisições | 2 | 1 |

Calibração: `scripts/calibrate_model_box.py <quadros> --current "cx cy cz sx sy sz"` (reproduz o centro
+0,09 m do X4). O X3 (`scenarios/roi_probe_x3.yaml`) é achatado demais para segmentar (~6 px de altura a
20–30 m): o deslocamento métrico em relação ao rastreador (−52 px·m) deu só a correção de centro
(+0,04 m). x500 e Tiltrotor usam a geometria sem o disco das hélices, ainda não medidos.
