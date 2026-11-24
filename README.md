# Perseguição Aérea Ar-Ar (aero_pursuit)

> **Branch `feat/mpc-from-scratch`:** as leis de controle, o gimbal, o treinamento por RL, o vento e a
> neblina foram removidos. Várias seções abaixo (§7 gimbal, §11 treinamento, leis de controle, `wind_node`,
> `fog_filter`, `training_arena`, `weather:`) descrevem o estado anterior e estão desatualizadas. O estado
> atual está em [`docs/MPC_PROPRIO.md`](docs/MPC_PROPRIO.md).

Drone perseguidor **x500_depth** (câmera RGB + profundidade fixa no corpo) ou **x500_gimbal_depth**
(a mesma câmera num **gimbal de 3 eixos**, [§7](#câmera-fixa-ou-gimbal)) (PX4 SITL + Pixhawk) que detecta, rastreia e persegue um drone
alvo em simulação **Gazebo Harmonic**, com **ROS 2 Jazzy**, visão por **TrackerVit (OpenCV DNN)**,
estimação por **Filtro de Kalman** e um conjunto de leis de controle intercambiáveis em execução
(MPC/LTV-MPC, LQR, LQI, PID-PBVS, ASMC, Hermite/planeza diferencial, guiamento PN/PP e política
externa para aprendizado por reforço). Operação e visualização pela **estação de solo web**
(`aero_pursuit_gcs`): vídeo de baixa latência, 3D, parâmetros, joystick e controle da simulação no
navegador, local ou remoto sem VPN ([§19](#19-estação-de-solo-web-gcs)). O Foxglove Studio segue opcional.

**Cenários** ([§20](#20-scenarios-and-target-paths)) tornam a perseguição 100% automática e
reproduzível: o caminho do alvo (pontos no mapa, formas, tabela CSV/Excel com tempo, comportamentos
reativos), aquisição automática do alvo pelo ground truth,
métricas de identidade do rastreador e estudos em lote ([§21](#21-estudos-em-lote)). A GCS tem editor
com waypoints no 3D e a câmera livre do Gazebo ([§22](#22-câmera-livre-do-gazebo)). Tudo roda também
num contêiner Docker ([§24](#24-docker)).

Resultados das investigações técnicas (parâmetros físicos do x500, formulação do LTV-MPC, MIGHTY,
GPU no Gazebo, TFs, logs): [docs/INVESTIGACOES.md](docs/INVESTIGACOES.md).

---

## Sumário

1. [Arquitetura](#1-arquitetura)
2. [Estrutura do repositório](#2-estrutura-do-repositório)
3. [Requisitos](#3-requisitos)
4. [Compilação](#4-compilação)
5. [Executando a simulação](#5-executando-a-simulação)
6. [Nós e scripts](#6-nós-e-scripts)
7. [Controladores](#7-controladores)
8. [Banda morta e métricas de desempenho](#8-banda-morta-e-métricas-de-desempenho)
9. [Seleção do alvo e modo omnisciente](#9-seleção-do-alvo-e-modo-omnisciente)
10. [Dataset (rosbag)](#10-dataset-rosbag)
11. [Treinamento: aprendizado por reforço](#11-treinamento-aprendizado-por-reforço)
12. [TFs e Foxglove](#12-tfs-e-foxglove)
13. [Joystick](#13-joystick)
14. [Logs e gerenciamento de processos](#14-logs-e-gerenciamento-de-processos)
15. [Renderização na GPU](#15-renderização-na-gpu)
16. [Depuração (gdb / VS Code)](#16-depuração-gdb--vs-code)
17. [Testes](#17-testes)
18. [Solução de problemas](#18-solução-de-problemas)
19. [Estação de solo web (GCS)](#19-estação-de-solo-web-gcs)
20. [Scenarios and target paths](#20-scenarios-and-target-paths)
21. [Estudos em lote](#21-estudos-em-lote)
22. [Câmera livre do Gazebo](#22-câmera-livre-do-gazebo)
24. [Docker](#24-docker)

---

## 1. Arquitetura

```
 Gazebo (lawn) ──ros_gz_bridge──► /camera, /depth_camera, /clock, /tf (PosePublisher)
      ▲                                  │
      │ PX4 SITL (x500_depth)            ▼
      │   ▲  ▲                    vision_tracker_node ──► /target_3d_position (câmera)
      │   │  └ MicroXRCEAgent           │  (TrackerVit + profundidade)  └► /target_bearing (sem profundidade)
      │   │    /fmu/in, /fmu/out        ▼
      │   │                     target_estimator_node ──► /target_3d_state_filtered (ENU, KF)
      │   │                             │                 └► /target_bearing_enu
      │   │                             ▼
      │   │                     pursuit_controller_node ──► /offboard/trajectory_setpoint
      │   │                     (supervisor + lei de controle   /pursuit/status, /pursuit/metrics,
      │   │                      + banda morta + limites)       /pursuit/predicted_path
      │   │                             ▼
      │   └──────────────────── offboard_control (decolagem, HOLD, repasse de setpoints)
      │
      ├── drone alvo (X4) ◄── drone_joy_teleop (joystick)  |  training_arena (evasor automático/RL)
      │
      ├── drones do cenário ◄── scenario_runner (gz-transport direto: spawn, twist, set_pose)
      │                          └► /scenario/status, /scenario/paths, TFs dos drones inseridos
      │                     target_acquisition: ROI pelo ground truth ──► /target_selection
      │                                         verdade do rastreador ──► /acquisition/truth
      └── spectator_camera (câmera livre) ──► /spectator/image ──► video_streamer_spectator ──► GCS

 session_recorder: grava cada engajamento (MCAP) em ~/aero_pursuit/datasets
```

Todos os nós (exceto `joy` e as TFs estáticas da câmera) rodam como **componentes** em um único
processo (`aero_pursuit_container`, multi-thread). Ponte Gazebo e visão usam comunicação
intra-processo: as imagens não são serializadas pelo DDS.

O supervisor de perseguição tem os modos `IDLE`, `TRACKING` (estimativa 3D recente), `COASTING`
(extrapola a estimativa por `coast_timeout`), `APPROACH` (só linha de visada, sem profundidade),
`LOST` (deixa de publicar; o `offboard_control` segura a posição) e `DISABLED`.

## 2. Estrutura do repositório

| Caminho | Conteúdo |
|---|---|
| `ros2_ws/src/aero_pursuit/` | Pacote principal (C++17 + módulo Python `aero_pursuit_rl`) |
| `ros2_ws/src/aero_pursuit/include,src/control/` | Leis de controle, supervisor, banda morta, métricas, planeza |
| `ros2_ws/src/aero_pursuit/include,src/{perception,estimation,learning}/` | Visão, KF e modelos de treinamento (sem ROS, testáveis) |
| `ros2_ws/src/aero_pursuit/include,src/nodes/` | Nós ROS 2 (componentes) |
| `ros2_ws/src/aero_pursuit/config/` | `aero_pursuit.yaml` (parâmetros), `gz_bridge_lawn*.yaml` (pontes) |
| `ros2_ws/src/aero_pursuit/launch/` | `aero_pursuit_sim.launch.xml` (simulação completa) |
| `ros2_ws/src/aero_pursuit/aero_pursuit_rl/` | Ambiente estilo Gymnasium para RL |
| `ros2_ws/src/aero_pursuit/gz_models/` | Modelos SDF dos alvos (X3, X4, Tiltrotor) e da câmera livre |
| `ros2_ws/src/aero_pursuit/include,src/scenario/` | Target paths, legs and the reference follower (no ROS, testable) |
| `ros2_ws/src/aero_pursuit/include,src/sim/` | Interface direta com o Gazebo (gz-transport) |
| `scenarios/` | Cenários YAML (`reference.yaml` documenta todos os campos) e estudos (`studies/`) |
| `docker/` | Imagem, compose e o atalho `aero.sh` |
| `ros2_ws/*.sh` | Scripts de simulação, diagnóstico e depuração (seção 6) |
| `.vscode/launch.json`, `tasks.json` | Depuração e build pelo VS Code |
| `docs/` | Investigações técnicas |
| `latex/`, `source/` | Texto do TCC e artigos de referência |

## 3. Requisitos

### Sem instalar nada: Docker

`./docker/aero.sh image && ./docker/aero.sh` sobe tudo num contêiner (detalhes em [§24](#24-docker)).

### Máquina nova: `install.sh`

```bash
git clone --recursive git@github.com:LuccasMarchetti/aero-pursuit.git ~/aero_pursuit
cd ~/aero_pursuit
./install.sh                 # tudo; pede sudo quando precisa (~40-60 min na primeira vez)
./install.sh --check         # diagnóstico: o que falta nesta máquina
./install.sh --dry-run       # só mostra os comandos
./install.sh --only ws       # só recompila o workspace (ou: ros, apt, px4, xrce, opencv, gcs)
./install.sh --gcs-service   # + GCS como serviço do usuário (sobe sozinha no login)
```

O script é idempotente (pula o que já existe) e instala: ROS 2 Jazzy + `ros_gz` (Gazebo Harmonic),
GStreamer com VA-API/x264/x265, PX4-Autopilot **fixado no commit testado** (`PX4_REF`), Micro-XRCE-DDS-Agent,
OpenCV 4.9 em `/usr/local`, o submódulo `px4_msgs`, o modelo ONNX e o `colcon build`. Funciona com o
clone em qualquer pasta (logs e datasets ficam na raiz do repositório). Para só **operar** a simulação
de outro computador basta um navegador (§19).

### Lista de dependências

- Ubuntu 24.04, **ROS 2 Jazzy**, **Gazebo Harmonic** (gz-sim 8) com `ros_gz` (`ros-jazzy-ros-gz`).
- **PX4-Autopilot** em `~/PX4-Autopilot` (ou `PX4_DIR=...`), compilável com `make px4_sitl`.
- **OpenCV ≥ 4.9** com `cv::TrackerVit` (instalado em `/usr/local`): `ros2_ws/install_opencv.sh`.
  Os avisos de conflito entre o OpenCV do sistema e o local durante o `colcon build` são esperados.
- `MicroXRCEAgent`, `ros-jazzy-foxglove-bridge`, `ros-jazzy-joy`, `tmux`, `gdb` (depuração).
- Modelo do TrackerVit: `ros2_ws/src/aero_pursuit/models/download_models.sh`.
- Opcional para RL: `pip install gymnasium` (e o framework de sua escolha, ex.: stable-baselines3).

## 4. Compilação

```bash
cd ~/aero_pursuit/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs        # uma vez
colcon build --packages-select aero_pursuit aero_pursuit_gcs
source install/setup.bash
```

A GCS roda direto do código-fonte pelo `gcs.sh` (mudanças na interface valem sem recompilar); o
`colcon build` do `aero_pursuit_gcs` só é necessário para `ros2 run aero_pursuit_gcs gcs_server`.

Build com símbolos de depuração: `./debug_attach.sh build` (RelWithDebInfo; o tipo fica no cache do
CMake até ser trocado de novo com `--cmake-args -DCMAKE_BUILD_TYPE=Release`).

## 5. Executando a simulação

### Pela GCS web (recomendado)

```bash
cd ~/aero_pursuit/ros2_ws
./gcs.sh                     # ou ./gcs.sh install-service (uma vez) para subir no login
```

Ou pelo comando `aero`, igual em todo computador que roda uma GCS (nativo ou em Docker):

```bash
./aero install               # GCS deste computador + ~/.local/bin/aero + início automático
aero view                    # abre esta GCS (sobe se preciso)
aero sim gazebo | isaac      # simulador desta GCS (Isaac: GPU RTX + aero isaac setup)
aero sim start | stop | status
aero status | aero access | aero gcs restart | aero update   # referência: docs/COMANDOS.md
```

No computador que só acessa, o lançador (`launcher/aero_launcher.py`, só Python) mostra as GCSs
que ele encontra na rede (local, ZeroTier, Tailscale) e abre a que você clicar.

Abra `http://localhost:8080` → aba **Simulação** → **Iniciar**. Todas as opções do launch e dos
scripts (controlador, yaw, joystick, gravação, treinamento, headless, build do PX4...) ficam no
formulário; parar, reiniciar, compilar, logs e console do PX4 também. Detalhes em [§19](#19-estação-de-solo-web-gcs).

### Sessão tmux (terminal)

```bash
cd ~/aero_pursuit/ros2_ws
./start_simulation.sh                                  # MPC (LTV), yaw voltado ao alvo
./start_simulation.sh controller:=lqi yaw_mode:=hold   # argumentos repassados ao launch
SKIP_PX4_BUILD=1 LOG_LEVEL=info ./start_simulation.sh  # sem recompilar o PX4, logs detalhados
```

Abre a sessão `px4_ros2_sim` com Gazebo + PX4, MicroXRCEAgent, Foxglove Bridge e o container do
`aero_pursuit`. Fechar a sessão encerra todos os processos.

### Gerenciador com um log por processo (alternativa)

```bash
./sim_manager.sh start controller:=hermite   # tudo em segundo plano
./sim_manager.sh status
./sim_manager.sh logs aero_pursuit pursuit_controller_node   # tail -f filtrado
./sim_manager.sh px4 "commander status"      # comando no console do PX4
./sim_manager.sh view                        # tmux só de leitura, um painel por log
./sim_manager.sh stop
```

### Argumentos do launch (`aero_pursuit_sim.launch.xml`)

| Argumento | Padrão | Descrição |
|---|---|---|
| `controller` | `mpc` | `mpc`, `lqr`, `lqi`, `pbvs_pid`, `asmc`, `hermite`, `hybrid_pn`, `external` |
| `yaw_mode` | `face_target` | `face_target`, `hold`, `velocity`, `fixed` |
| `camera_mount` | `$AERO_CAMERA_MOUNT` ou `fixed` | `fixed` (x500_depth) ou `gimbal` (x500_gimbal_depth + `gimbal_controller`); deve bater com o modelo do Gazebo |
| `gimbal_mode` / `gimbal_inertia` | `track` / `true` | Modo inicial do gimbal e inércia da câmera (ambos trocáveis em execução) |
| `dnn_target` | `opencl` | Backend do TrackerVit: `cpu`, `opencl`, `opencl_fp16` |
| `tf_source` | `gazebo` | TFs dos modelos: `gazebo` (PosePublisher + ponte) ou `node` (`gazebo_tf_bridge`) |
| `record` / `record_mode` | `true` / `session` | Gravação do dataset (`session` ou `continuous`) |
| `dataset_dir` | `~/aero_pursuit/datasets` | Destino dos bags |
| `training` / `evader_policy` | `false` / `flee` | Arena de treinamento e política do evasor |
| `joystick` / `joy_driver` | `true` / `game_controller_node` | Driver do joystick do alvo |
| `log_level` | `warn` | Nível de log de todos os nós (`debug`, `info`, `warn`, `error`) |
| `output` | `screen` | `log` envia a saída só para `~/.ros/log` |
| `opencv_log_level` | `ERROR` | Verbosidade do OpenCV |
| `debug` / `gdb_port` | `false` / `3000` | Container sob `gdbserver` |
| `container_prefix` | (derivado de `debug`) | Prefixo livre do processo (ex.: `valgrind`) |
| `world_name`, `params_file`, `bridge_config`, `tf_bridge_config`, `vittrack_model_path` | | Caminhos/mundo |

Variáveis de ambiente dos scripts: `WORLD_NAME`, `SKIP_PX4_BUILD`, `PX4_DIR`, `LOG_LEVEL`,
`XRCE_VERBOSE` (0–6, padrão 3), `GZ_SIM_VERBOSE` (padrão 1; `GZ_VERBOSE` é aceito como sinônimo,
mas não é exportado: no ambiente ele liga o modo debug do gz-transport), `GZ_TRANSPORT_VERBOSE=1`,
`GZ_RENDER_API` (`opengl`/`vulkan`), `GZ_HEADLESS=1`, `GZ_DRI_PRIME=1`, `PX4_QUIET=1`,
`FOXGLOVE_ALL_TOPICS=1` (anuncia as imagens cruas no Foxglove), `AERO_CAMERA_MOUNT=fixed|gimbal`
(câmera do perseguidor: lida pelo `launch_gz_and_drones.sh`, que insere o modelo, e pelo launch).

Encerramento: Ctrl+C no `launch_gz_and_drones.sh` (PX4 → Gazebo → SIGKILL do que sobrar), fechar a
sessão tmux, ou `./stop_simulation.sh` (`DRY_RUN=1` só lista o que seria encerrado).

## 6. Nós e scripts

### Nós (componentes do pacote `aero_pursuit`)

| Nó | Função | Entradas → Saídas principais |
|---|---|---|
| `vision_tracker_node` | Rastreia o alvo na RGB (TrackerVit), mede a profundidade (percentis na caixa) | `/camera`, `/depth_camera` → `/target_3d_position`, `/target_bearing`, `/target_detection`, `/camera/segmented_bb[/compressed]` |
| `target_estimator_node` | KF de velocidade constante no referencial `x500_depth_0_enu`; gating χ²; modo omnisciente (TF do Gazebo) | `/target_3d_position`, `/fmu/out/vehicle_attitude` → `/target_3d_state_filtered`, `/target_bearing_enu`, `/target_marker`, TF `target_estimate` |
| `pursuit_controller_node` | Supervisor + lei de controle + banda morta + limites + yaw | `/target_3d_state_filtered`, `/target_bearing_enu` → `/offboard/trajectory_setpoint`, `/pursuit/{status,metrics,predicted_path}` |
| `offboard_control` | Máquina de estados Offboard do PX4 (decolagem, HOLD, repasse) | `/offboard/trajectory_setpoint` → `/fmu/in/*` |
| `session_recorder` | Grava cada engajamento em MCAP + `session_info.yaml` com métricas | `/pursuit/status`, `/pursuit/metrics` + lista de tópicos |
| `training_arena` | Episódios perseguidor × evasor, recompensa, política do evasor | TF → `/training/observation`, `/X4/gazebo/command/twist` |
| `drone_joy_teleop` | Joystick → velocidade do alvo | `/joy` → `/X4/gazebo/command/twist` |
| `gazebo_tf_bridge` | (Alternativo) poses do Gazebo → TF | `/world/<w>/dynamic_pose/info` → `/tf` |
| `scenario_runner` | Executa cenários: insere drones, estágio, planos, engajamento, repetições | `~/command` → `/scenario/status`, `/scenario/paths`, TFs dos drones inseridos |
| `target_acquisition` | ROI automática pelo ground truth (8 cantos da caixa visível orientada, TF no instante do quadro); IoU/identidade do rastreador | `/target_detection`, TF, `/camera/camera_info` → `/target_selection`, `/acquisition/truth` |
| `spectator_camera` | Câmera livre do Gazebo (pose livre, seguir, orbitar) | `/spectator/command` → `set_pose`, TF `spectator_camera` |
| `wind_node` | Vento: parâmetros `wind.*`, carrega o sistema `aero_wind` no Gazebo (força em cada drone a cada passo) | parâmetros → `/world/<w>/aero_wind/config`; estado → `/weather/wind` |
| `gimbal_controller` | (Só com `camera_mount:=gimbal`) Gimbal de 3 eixos: aponta a câmera para o alvo, estabiliza ou trava; inércia opcional; TF dinâmica da câmera | `/target_3d_state_filtered`, `/target_bearing_enu`, `/fmu/out/vehicle_attitude`, juntas do Gazebo → `/model/x500_depth_0/gimbal/{yaw,roll,pitch}`, TF `gimbal_mount → camera_link`, `/gimbal/{state,command}` |
| `fog_filter` | Neblina na câmera (Koschmieder com a profundidade); repassa sem cópia quando desligada | `/camera/clear`, `/depth_camera/clear` → `/camera`, `/depth_camera` |

Serviços: `/pursuit_controller_node/enable` (`std_srvs/SetBool`), `/training_arena/reset_episode`
(`std_srvs/Trigger`), `/world/lawn/control` e `/world/lawn/set_pose` (Gazebo, via ponte),
`/scenario_runner/command` (`aero_pursuit/srv/ScenarioCommand`: load, start, stop, pause, resume, unload).

### Scripts (`ros2_ws/`)

| Script | Função |
|---|---|
| `start_simulation.sh` | Sessão tmux completa (seção 5) |
| `sim_manager.sh` | Processos em segundo plano com log por processo (seção 14) |
| `launch_gz_and_drones.sh` | Gazebo + modelos alvo + PX4 SITL; opções de GPU; anexa as TFs |
| `gz_pose_tf.sh` | Anexa o `PosePublisher` aos modelos em execução (TFs nativas) |
| `check_gpu_rendering.sh` | Diagnóstico: o Gazebo está usando a GPU? |
| `omniscient_tracker.sh [frame] [true\|false]` | Alterna a fonte de medida para o ground truth |
| `debug_attach.sh {build\|attach\|server\|core}` | Depuração com gdb |
| `install_opencv.sh` | Compila/instala o OpenCV com TrackerVit |
| `run_scenario.sh <cenário>` | Simulação headless executando um cenário de `scenarios/` |
| `run_study.py <estudo.yaml>` | Estudo em lote (§21) |
| `tune_study.py <sintonia.yaml>` | Sintonia em lote no SITL (Nelder–Mead em log θ, §21) |
| `identify_px4_lag.py <sessões>` | τ e tempo morto da malha de velocidade do PX4 a partir dos logs (§21) |
| `scenario_env.sh` | (Library) resolves and validates `SCENARIO=` for the scripts |
| `ros2 run aero_pursuit export_dataset.py <bag\|dir>` | Exporta sessões para CSV |
| `ros2 run aero_pursuit rl_random_agent.py` | Exemplo de agente de RL |

## 7. Controladores

| `controller_type` | Lei | Destaque |
|---|---|---|
| `mpc` | MPC sobre o duplo integrador 3D | `mpc.online: true` (padrão) = **LTV-MPC**: QP remontado e resolvido a cada ciclo, grade de tempo crescente, referência na LOS prevista, restrições de aceleração e velocidade. `false` = ganho fixo pré-computado |
| `lqr` | LQR de horizonte infinito | Ganho da DARE offline; referência para o MPC |
| `lqi` | LQR com ação integral | Erro de regime nulo sob perturbações constantes (vento, viés do laço do PX4) |
| `pbvs_pid` | PID servovisual (PBVS) + feedforward | Anti-windup por zona e saturação |
| `asmc` | Modos deslizantes adaptativo | Ganho chaveado adaptativo, camada-limite contra *chattering* |
| `hermite` | Replanejamento com spline de Hermite quíntica (MIGHTY) + planeza diferencial | Otimiza a duração do plano (tempo × jerk × limites) e envia **feedforward de jerk** ao PX4 |
| `hybrid_pn` | Guiamento híbrido PN/PP | Navegação proporcional + perseguição pura + manutenção de distância |
| `external` | Política externa (RL) | Velocidade em `/training/pursuer_action`; supervisor e limites continuam ativos |

### Como alternar

```bash
./start_simulation.sh controller:=lqi                         # na partida
ros2 param set /pursuit_controller_node controller_type asmc  # em execução
ros2 param set /pursuit_controller_node mpc.online false      # MPC de ganho fixo
ros2 param set /pursuit_controller_node mpc.horizon_N 15
ros2 param set /pursuit_controller_node yaw_mode velocity
ros2 service call /pursuit_controller_node/enable std_srvs/srv/SetBool "{data: false}"
```

Valores inválidos são rejeitados. Mudanças de controlador, ganhos, `target_distance`, `max_vel` e
`max_acc` recriam a lei (zerando integradores/warm start); yaw, fallback, banda morta e timeouts são
aplicados sem reset. Todos os parâmetros estão comentados em
[`config/aero_pursuit.yaml`](ros2_ws/src/aero_pursuit/config/aero_pursuit.yaml).

A trajetória prevista (MPC) ou planejada (Hermite) é publicada em `/pursuit/predicted_path`.

### Saída para o PX4 (`output`)

`output`: `trajectory` (padrão, velocidade + aceleração), `attitude` (atitude + empuxo), `thrust_torque`
(empuxo + torque normalizados; o PX4 só mistura) ou `motors` (ActuatorMotors, empuxo de cada motor).
As saídas diretas existem só na família MPC: MPC cinemático e `ibvs_mpc` aceitam as quatro, o MPC dinâmico
`attitude`/`thrust_torque`/`motors` (o modelo dele já tem a dinâmica de atitude); as demais leis só
`trajectory` (o nó recusa outra, e a GCS nem oferece). `mpc.lag.*` modela o atraso da malha interna em
toda a família (velocidade no cinemático e no IBVS, atitude no dinâmico); no dinâmico, `vertical_mode`
entra como referência de altitude dentro do MPC. Tabela completa no
[manual](docs/MANUAL_GCS.md#31-saída-para-o-px4-output). Nos modelos cinemáticos
(`kinematic`, com ou sem `mpc.lag`, e `ibvs_mpc`) uma malha de velocidade do próprio nó (`velocity_loop.*`,
padrões da PX4) substitui a da PX4 e a atitude sai por planicidade. Em `thrust_torque`/`motors` um laço de
atitude em SO(3) do próprio nó roda na taxa da odometria e a alocação usa o modelo físico do x500
(`MotorAllocation`). Erro de atitude acima de 60° ou referência velha => o offboard volta a segurar
posição. Detalhes no [manual](docs/MANUAL_GCS.md#31-saída-para-o-px4-output).

### Centralização do alvo (yaw e canal vertical)

`yaw_mode: face_target` centraliza o alvo na horizontal. O canal vertical é escolhido por
`vertical_mode` (ao vivo, na partida com `vertical_mode:=...` ou na aba Perseguição da GCS):

| `vertical_mode` | Comportamento |
|---|---|
| `los` (padrão) | A lei segue o standoff ao longo da LOS 3D (mantém o ângulo de elevação, não iguala a altitude) |
| `match_altitude` | `v_z = v_alvo,z + k·(Δz + vertical.altitude_offset)`: mesma altitude do alvo |
| `center_image` | Altitude que põe o alvo no centro vertical da imagem (montagem da câmera + arfagem medida, filtrada) |

Substitui só o canal z do comando de qualquer lei (exceto o MPC dinâmico com saída direta). Detalhes no
[manual da GCS](docs/MANUAL_GCS.md#3-modos-de-controle-centralização-do-alvo).

### Câmera fixa ou gimbal

O perseguidor pode voar com a OakD-Lite (RGB IMX214 + profundidade StereoOV7251) **fixa no corpo**
(`x500_depth` do PX4, **padrão**) ou num **gimbal de 3 eixos** (yaw, roll, pitch) sob a frente do corpo
(modelo `ros2_ws/src/aero_pursuit/gz_models/x500_gimbal_depth`). O modelo com gimbal usa a mesma câmera,
então a **profundidade continua alinhada com a imagem para onde o gimbal olha**.

| | Câmera fixa (`fixed`) | Gimbal (`gimbal`) |
|---|---|---|
| Modelo no Gazebo | `x500_depth` (inserido pelo PX4) | `x500_gimbal_depth`, inserido com o nome `x500_depth_0`; o PX4 se conecta a ele (`PX4_GZ_MODEL_NAME`, mesmo airframe 4002) |
| TF da câmera | estática `x500_depth_0 → camera_link` | estática `x500_depth_0 → gimbal_mount` + **dinâmica** `gimbal_mount → camera_link` (ângulos medidos das juntas, no carimbo da simulação) |
| Centralizar o alvo | gira o corpo (`yaw_mode: face_target`) e muda a altitude (`center_image`) | o gimbal mantém o alvo no centro; o corpo só cuida da distância |

**Escolha do modelo** (na partida; trocar exige reiniciar a simulação): GCS → Simulação → *Câmera do
perseguidor*, `AERO_CAMERA_MOUNT=gimbal ./sim_manager.sh start` ou
`AERO_CAMERA_MOUNT=gimbal ./start_simulation.sh`. Frames, tópicos e o bridge são os mesmos nos dois.

**Em execução** (nó `gimbal_controller`, GCS → Perseguição → *Gimbal da câmera*):

| Parâmetro | Padrão | Efeito |
|---|---|---|
| `mode` | `track` | `track`: eixo óptico na linha de visada do alvo (estimativa filtrada, ou a LOS ENU sem profundidade); sem alvo segura a última direção por `track.lost_hold_s` e depois estabiliza. `stabilize`: horizonte nivelado, rumo do corpo, elevação `neutral_pitch_deg`. `locked`: juntas paradas em (0, 0, `neutral_pitch_deg`) = **câmera fixa** no modelo com gimbal. `mpc`: segue o setpoint do IBVS-MPC com `ibvs.gimbal_in_loop` (`/gimbal/pointing_setpoint`); sem setpoint recente, age como `track` |
| `inertia.enabled` | **`true` (ligada)** | Movimento da câmera com velocidade (`inertia.max_rate_deg_s`, 90°/s) e aceleração (`inertia.max_accel_deg_s2`, 360°/s²) limitadas, perfil sem sobressinal e *feedforward* da taxa do alvo (`inertia.feedforward_tau`). Desligada: a câmera salta para o ângulo pedido (sobra só a malha da junta no Gazebo, τ ≈ 25 ms) |
| `neutral_pitch_deg` | `0` | Elevação de `locked`/`stabilize`; também é a referência do `center_image` com gimbal (TF `gimbal_mount → gimbal_neutral`) |
| `attitude_source` | `auto` | Atitude usada para apontar: no modo omnisciente a verdade do Gazebo, com visão o EKF do PX4 (a mesma do estimador; o rumo do EKF chega a diferir ~8° do Gazebo) |
| `limits.*` | yaw ±170°, roll ±45°, pitch −90…+30° | Curso das juntas |

A estabilização (rejeitar a rotação do corpo) é feita no espaço do mundo; a inércia vale para o
movimento de apontamento. Com `mode: track`, combine `yaw_mode: hold` ou `velocity` para manter a
distância **sem girar o corpo**; `face_target` continua válido (o gimbal só corrige o resto).
Medido no SITL (alvo a 5 m andando a 1,5 m/s de lado, corpo com yaw fixo): erro do eixo óptico < 1°
no omnisciente e < 1,1° com visão (gimbal girou 61° em yaw).

## 8. Banda morta e métricas de desempenho

**Banda morta** (`deadband.*`, vale para qualquer controlador): com erro normalizado
`ε = max(|e_p|/tol_p, |e_v|/tol_v)` abaixo de 1, o fator de atuação α vai suavemente a 0 (rampa
*smoothstep* + filtro de 1ª ordem) e o comando passa a ser só o feedforward da velocidade do alvo:
`v = v_alvo + α (v_lei − v_alvo)`, `a = α a_lei`. Isso elimina a realimentação de alta frequência
quando o perseguidor já está na posição desejada.

```bash
ros2 param set /pursuit_controller_node deadband.enabled true
ros2 param set /pursuit_controller_node deadband.position_tolerance 0.5
```

**Métricas** (`/pursuit/metrics`, `aero_pursuit/msg/PursuitMetrics`, a cada ciclo ativo):
erro de posição ao ponto de standoff, erro de velocidade, erro de distância, **erro angular da LOS**
(rumo × azimute do alvo) e desvio ao eixo óptico, aceleração, fator da banda morta, empuxo/inclinação
exigidos (planeza diferencial com os parâmetros do x500), tempo de cálculo da lei e os acumulados
desde o engajamento: **RMSE** de posição/velocidade/LOS, erro máximo, **esforço de controle**
`∫|a|²dt` e `∫|a|dt`. O tópico vai para o rosbag e o resumo final para o `session_info.yaml`.

## 9. Seleção do alvo e modo omnisciente

1. No Foxglove, abra um painel *Image* em `/camera/segmented_bb/compressed`.
2. Clique em um canto do alvo e depois no canto oposto (o *hover* mostra a pré-visualização).
3. O TrackerVit inicia; após `min_consecutive_frames` quadros com score acima do limiar, a posição 3D
   é publicada e o controlador entra em `TRACKING`.

Sem pontos de profundidade, a visão publica só a linha de visada e o supervisor entra em `APPROACH`
(aproximação alinhando o yaw primeiro). A câmera de profundidade mede de 0,2 a 19,1 m (medido: válida
até ~19 m, +inf além). Com a ROI toda além do alcance, a linha de visada leva `range_lower_bound =
19,1 m` e o APPROACH nunca usa uma distância menor que essa.

**Ground truth** (sem visão): `./omniscient_tracker.sh X4 true` (volta com `false`). O modo
omnisciente publica a pose relativa exata das TFs do mundo, sem filtro; a velocidade é a diferença
entre poses consecutivas.

**Física e clima**: modelo aerodinâmico (`/wind_node` `aero.*` → `aero_gz_physics`), vento (`wind_node`
+ sistema `aero_wind` do Gazebo) e neblina (`fog_filter`) ao vivo pela aba Física e clima da GCS, por
`ros2 param set /wind_node wind.enabled true` ou pelos blocos `physics:` e `weather:` dos cenários.
Detalhes em `docs/MANUAL_GCS.md` (2.5.1).

## 10. Dataset (rosbag)

Com `record:=true`, cada engajamento vira um bag MCAP em `~/aero_pursuit/datasets/<data>_<controlador>/`
(com pré e pós-roll), contendo estados, setpoints, TFs, detecções, métricas e observações de
treinamento (imagens via `session_recorder.extra_topics`). `session_info.yaml` traz duração, tempo
em cada modo, contagem de mensagens e as métricas finais.

```bash
ros2 run aero_pursuit export_dataset.py ~/aero_pursuit/datasets     # CSV por tópico
```

## 11. Treinamento: aprendizado por reforço

A arena (`training:=true`) delimita episódios (captura < 1,5 m, fuga > 60 m, timeout 120 s), calcula a
recompensa de soma zero e publica `/training/observation`. O módulo `aero_pursuit_rl` expõe o
ambiente com API Gymnasium:

```python
from aero_pursuit_rl.env import PursuitEvasionEnv

env = PursuitEvasionEnv(agent='pursuer', step_mode='lockstep')  # 'evader' | 'both'
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

- `agent`: `pursuer` (configura `controller_type=external` e o modo omnisciente), `evader`
  (`evader_policy=external`) ou `both` (self-play; ação de 6 elementos, recompensa `(r, −r)`).
- `step_mode`: `realtime` (sincroniza com a arena a 20 Hz) ou `lockstep` (mundo pausado e avançado
  exatamente `dt` por passo via `/world/lawn/control` — determinístico, independe da carga da CPU).
- Ações normalizadas em [−1, 1] × `max_speed`; observação de 16 elementos (`spaces.OBSERVATION_NAMES`).
- No `reset`, o evasor é reposicionado aleatoriamente ao redor do perseguidor (`/world/lawn/set_pose`).

```bash
./start_simulation.sh training:=true record:=false
ros2 run aero_pursuit rl_random_agent.py --agent both --step-mode lockstep
```

As conversões de `aero_pursuit_rl/spaces.py` não dependem de ROS e podem montar o mesmo vetor a
partir dos bags gravados (aprendizado por imitação / offline RL).

## 12. TFs e Foxglove

Árvore de TFs (todas com `frame_id` e `child_frame_id` definidos):

```
lawn ─┬─ x500_depth_0 ─ camera_link ─ camera_optical      (PosePublisher / estáticas do launch)
      ├─ X3, X4, Tiltrotor                                (PosePublisher)
x500_depth_0 ─ x500_depth_0_enu ─ target_estimate          (target_estimator_node)
lawn ─┬─ <drones inseridos pelo cenário>, x500_depth_<i>   (scenario_runner)
      └─ spectator_camera                                 (spectator_camera)
```

Os drones que já têm `PosePublisher` (X3, X4, Tiltrotor, x500_depth_0) não são republicados pelo
`scenario_runner`: cada TF tem uma única fonte. `/scenario/paths` (MarkerArray) mostra no painel 3D
do Foxglove os percursos previstos de cada drone do cenário.

Por padrão (`tf_source:=gazebo`) as poses vêm do sistema `PosePublisher` do Gazebo, anexado em tempo
de execução por `gz_pose_tf.sh`, e o `ros_gz_bridge` converte `gz.msgs.Pose_V` em
`tf2_msgs/TFMessage` — sem nó tradutor. `tf_source:=node` volta ao `gazebo_tf_bridge`.

Foxglove: conecte em `ws://localhost:8765`; painéis úteis: *3D* (frame fixo `lawn`, TFs,
`/pursuit/predicted_path`, `/target_marker`), *Image* (`/camera/segmented_bb/compressed`) e *Plot*
(`/pursuit/metrics.rmse_position`, `.los_error`, `.deadband_factor`, `.solve_time_us`).

### Foxglove remoto / baixa latência

O bridge usa `config/foxglove_bridge.yaml`. Atraso de vários segundos = fila: o bridge acumula até
`send_buffer_limit` por cliente (padrão 10 MB) e o kernel mais alguns MB por socket; com o link mais
lento que o pedido pelos painéis, tudo (imagem, TF, status) chega atrasado. O yaml:

- não anuncia `/camera` (1920×1080 RGB, ~1,5 Gbit/s), `/depth_camera` nem `/camera/segmented_bb` cru;
- limita a fila a ~1 MB (o bridge descarta em vez de acumular);
- o `vision_tracker_node` publica o JPEG de debug a 15 Hz, qualidade 60 (~3–5 Mbit/s).

Ajuste opcional no kernel (fila do socket TCP só com o necessário, menos atraso em link lento):

```bash
sudo sysctl -w net.ipv4.tcp_notsent_lowat=131072   # persistir: /etc/sysctl.d/99-foxglove.conf
```

Em link ruim, reduza mais: `ros2 param set /vision_tracker_node debug_image_rate_hz 10` /
`debug_jpeg_quality 40`. No Foxglove, evite painéis assinando tópicos que não está olhando.

## 13. Joystick

O `drone_joy_teleop` comanda um veículo por vez: os alvos de `target_vehicles` (padrão X4, X3,
Tiltrotor; Twist em `/<nome>/gazebo/command/twist`) e o perseguidor. LB/RB (teclado da GCS: `[` `]`,
`,` `.`, PgUp/PgDn) percorrem a lista; `/drone_joy_teleop/select` (`std_msgs/String`: nome, `chaser`,
`next`, `prev`) seleciona direto. Com `idle_silence: true` (padrão) manches neutros não publicam nada
(só um comando nulo ao soltar), para não sobrepor cenários e o evasor. Mapeamento (modo 2 de RC) no
layout de eixos do `joy game_controller_node` (0 LX, 1 LY, 2 RX, 3 RY):

| Comando | Eixo | Sinal |
|---|---|---|
| Frente/trás (`linear.x`) | 3 – analógico direito vertical | invertido |
| Lateral (`linear.y`) | 2 – analógico direito horizontal | invertido |
| Subir/descer (`linear.z`) | 1 – analógico esquerdo vertical | invertido |
| Yaw (`angular.z`) | 0 – analógico esquerdo horizontal | invertido |

Ajustes sem recompilar: `axis_*`, `invert_*`, `max_*` (parâmetros do nó). Outro driver:
`joy_driver:=joy_node` (layout cru do dispositivo; para um controle Xbox o eixo 2 é o gatilho LT).
Para usar o seu próprio driver: `joystick:=false` e rode-o à parte.

Sem controle físico, a GCS publica o mesmo `/joy` pelo teclado ou por um **joystick virtual na tela**
(botão 🎮 no cabeçalho): dois manches de toque no mesmo mapeamento acima, mais ◀/▶ (LB/RB) e
velocidade −/+ (LT/RT). É o jeito de pilotar pelo celular ([manual da GCS §2.4](docs/MANUAL_GCS.md#24-joystick)).

## 14. Logs e gerenciamento de processos

- Nível padrão **WARN** para todos os nós ROS (`log_level:=info` para depurar); mensagens de
  rotina (criação das pontes, parâmetros aplicados, sessões abertas) não aparecem no terminal.
- MicroXRCEAgent com `-v 3`, Gazebo com `-v 1`, Foxglove Bridge em WARN; `PX4_QUIET=1` desvia o
  console do PX4 para arquivo.
- `sim_manager.sh` grava cada processo em `~/aero_pursuit/logs/<data_hora>/<processo>.log` e os logs
  por processo do ROS 2 em `.../ros/` (`ROS_LOG_DIR`). `logs <processo> <filtro>` segmenta por nó,
  já que todos os componentes compartilham o processo do container.
- Restam apenas ~20 linhas na partida/encerramento (do `ros2 launch`) e 3 avisos únicos do OpenCL do
  OpenCV durante a autotunagem do TrackerVit.

## 15. Renderização na GPU

Os sensores (RGB 1920×1080 e profundidade 640×480, 30 Hz) são renderizados pelo sistema `Sensors`
com **OGRE2**. Verificado nesta máquina (Radeon RX 6600, Mesa): o processo do Gazebo é cliente DRM
`amdgpu` tanto com GUI quanto em modo headless (EGL).

```bash
./check_gpu_rendering.sh                  # com a simulação rodando
GZ_HEADLESS=1 ./start_simulation.sh       # sem GUI, sensores na GPU via EGL
GZ_RENDER_API=vulkan ./start_simulation.sh  # experimental; volta a OpenGL se o plugin estiver quebrado
```

Detalhes e limitações (Vulkan do `gz_ogre_next_vendor`, física na CPU): `docs/INVESTIGACOES.md`.

## 16. Depuração (gdb / VS Code)

| Forma | Como |
|---|---|
| Anexar ao container em execução | `./debug_attach.sh attach` ou VS Code → *aero_pursuit: anexar ao container*. Requer `sudo sysctl -w kernel.yama.ptrace_scope=0` |
| Container sob gdbserver | `./start_simulation.sh debug:=true` e VS Code → *aero_pursuit: gdbserver :3000* (o container espera o depurador; a porta escuta em todas as interfaces) |
| Nó isolado | VS Code → *pursuit_controller_node isolado* |
| Testes | VS Code → *testes de controle (gtest)* |
| Core dump | `./debug_attach.sh core` (`coredumpctl`) |

Compile antes com `./debug_attach.sh build` (ou a tarefa *colcon: build RelWithDebInfo*). Breakpoints
por função funcionam em qualquer componente, ex.:
`break aero_pursuit::PursuitControllerNode::runController`.

## 17. Testes

```bash
cd ~/aero_pursuit/ros2_ws
colcon test --packages-select aero_pursuit && colcon test-result --verbose
```

Cobertura: convergência de todas as leis contra alvo em movimento, LTV-MPC (equivalência com o MPC
fixo, restrições), LQI × LQR sob perturbação, ASMC, spline de Hermite (condições nodais, custo de
jerk fechado, mapa Bézier), planeza diferencial (x500), banda morta, métricas, supervisor, yaw,
referenciais, KF, câmera, TrackerVit, arena e conversões do ambiente de RL (pytest).

## 18. Solução de problemas

| Sintoma | Causa / solução |
|---|---|
| Container "trava" ao encerrar e bag sem `metadata.yaml` | Segfault do `RosGzBridge` no `dlclose`; corrigido com `LD_PRELOAD` no launch. Se reaparecer, confira `$(ros2 pkg prefix ros_gz_bridge)/lib/libros_gz_bridge.so` |
| Joystick com eixos trocados | Driver diferente do esperado: `joy_driver:=game_controller_node` (padrão) ou ajuste `axis_*`/`invert_*` |
| Foxglove com segundos de atraso | Imagem crua ou taxa alta para o link: ver *Foxglove remoto* (§12) |
| PX4 `gz_bridge failed to start and spawn model` (`world: default`) | Gazebo antigo ainda rodando ou mundo errado; `./stop_simulation.sh` e reinicie |
| X4/X3/Tiltrotor ignoram o joystick | O `MulticopterVelocityControl` começa desligado: o `launch_gz_and_drones.sh` o liga na partida; à mão, aba Perseguição → Drones-alvo → Ligar (`/<ns>/enable`) |
| GCS: "WebCodecs indisponível" | Navegador fora de contexto seguro: use `http://localhost`, o `https://<ip>:8443` (aceite o certificado) ou o túnel; ou o codec MJPEG |
| GCS: H.265 "sem suporte" | O navegador não decodifica HEVC (no Linux o Chrome só o faz com GPU): use H.264 |
| GCS não vê a simulação | Mesmo `ROS_DOMAIN_ID` nos dois lados; o `gcs.sh` herda o do terminal/serviço |
| TFs dos modelos ausentes | Veja `~/aero_pursuit/logs/latest/gz_pose_tf.log`; rode `./gz_pose_tf.sh lawn` manualmente ou use `tf_source:=node` |
| Câmeras lentas / CPU alta | `./check_gpu_rendering.sh`; sem cliente DRM => driver de software (llvmpipe) |
| `ros2 run` não encerra com `kill -INT <pid>` | O `ros2 run` repassa o Ctrl-C do terminal (grupo de processos); use `kill -INT -<pgid>` |
| Controlador não engaja | `ros2 topic echo /pursuit/status` (modo e idade da estimativa); selecione o alvo ou use o modo omnisciente |
| Cenário parado em STAGING | Algum drone não chegou à partida (`staging.tolerance`) ou o PX4 não decolou; após `staging.timeout` a execução começa assim mesmo. `ros2 topic echo /scenario/status` mostra o estado de cada um |
| Muitas linhas `OpenCL error CL_INVALID_WORK_GROUP_SIZE` | O backend OpenCL do OpenCV DNN falha nesta GPU e recai na CPU a cada quadro; com rastreamento contínuo (cenários) o log cresce. Use `dnn_target:=cpu` |

## 19. Estação de solo web (GCS)

Pacote `aero_pursuit_gcs` (servidor Python/aiohttp + ponte rclpy) e dois componentes C++ no
container: `video_streamer` (vídeo) e `gcs_telemetry` (poses). Não há etapa de build da interface:
HTML/JS puros servidos pelo próprio servidor.

```
Gazebo ─(ros_gz_bridge, intra-processo, sem cópia)─► video_streamer ─ GStreamer: escala ► VA-API/x264/x265/JPEG
                                                          │ socket Unix (64 B de cabeçalho + quadro)
TFs ─► gcs_telemetry (C++, 30 Hz) ─┐                       ▼
tópicos/serviços/parâmetros ───────┴──► gcs_server (Python) ─ WebSocket ─► navegador
                                                                 (WebCodecs num Worker, three.js)
```

### Acesso

| De onde | Endereço | Autenticação |
|---|---|---|
| Esta máquina | `http://localhost:8080` | dispensada (desligável na aba Rede) |
| Rede local | `https://<ip>:8443` (certificado autoassinado: aceite o aviso uma vez) | token |
| Qualquer lugar, sem VPN | aba Rede → **Iniciar túnel** (Cloudflare, gratuito, sem conta) → link `https://*.trycloudflare.com` | token |

O token fica em `~/.config/aero_gcs/settings.json` (`./gcs.sh token`); "Copiar link com token" já
autentica o navegador remoto. O túnel rápido gera uma URL nova a cada início; para URL fixa use um
túnel nomeado (conta gratuita da Cloudflare + domínio) e cole o token na aba Rede. IP fixo não é
necessário (e com CGNAT do provedor nem funcionaria).

### Abas

Guia de uso completo, aba por aba: [`docs/MANUAL_GCS.md`](docs/MANUAL_GCS.md) (resumo na aba **Ajuda**).

- **Perseguição**: estado do supervisor e do rastreador, habilitar/desabilitar, controlador/
  distância/velocidades e **centralização (yaw + vertical)** ao vivo, parar rastreamento, modo
  omnisciente. **Seleção do alvo: arraste sobre o vídeo** (botão direito ou Esc para parar),
  publicada em `/target_selection` em pixels da imagem original.
- **Drones**: veículo comandado pelo joystick/teclado (um botão por drone), drones-alvo (ligar/
  desligar/parar), mundo (pausar/passo/reset), reinício de episódio.
- **Vídeo**: fonte (`/camera`, `/camera/segmented_bb`, `/depth_camera` colorizada ou qualquer
  `Image`), codec (H.264/H.265/MJPEG), GPU/CPU, **resolução 10–100% da original**, fps, bitrate
  (teto), modo adaptativo, CBR/VBR/CQP, QP/qualidade JPEG, GOP e quadros em trânsito — tudo ao vivo.
- **Joystick**: controle do navegador (Gamepad API) ou teclado → `/joy` no layout do
  `game_controller_node`; zona morta, taxa, RTT do comando. Sem pacotes por 0,5 s o servidor publica
  comando neutro. Inicie a simulação com "Joystick: navegador" para não conflitar com um USB local.
- **Parâmetros**: qualquer nó; sliders, chaves e listas, sincronizados por `/parameter_events`.
- **Física e clima**: modelo aerodinâmico ao vivo (`/wind_node` `aero.*`: corpo none/linear/quadratic/
  panels/table, BEMT, efeito solo, VRS, esteira), vento e neblina.
- **Layout modular**: câmera, vistas e painel são janelas arrastáveis/redimensionáveis; qualquer aba pode
  virar janela própria (⧉); presets e modo clássico no menu *Layout* (manual §0).
- **Simulação**: iniciar/parar/reiniciar com todas as opções, compilar, processos (CPU/RAM), logs ao
  vivo com filtro, console do PX4. Também: **simulador** desta GCS (Gazebo/Isaac Sim, reinicia a GCS),
  **perseguidor remoto** (Raspberry Pi por `docker/pursuer.sh`, estado e latência do host, parada por
  latência; `docs/RASPBERRY.md`) e **replay de sessão** do rosbag de `datasets/`: *no simulador*
  (Gazebo sem perseguidor nem PX4; o nó `world_replay` teleporta os modelos pelas poses gravadas e a
  câmera do perseguidor é renderizada de novo) ou *só a gravação* (3D, gráficos e status, também no
  Isaac); pausa, velocidade 0,25–4× e busca; sem os comandos aos veículos ([manual §7](docs/MANUAL_GCS.md#7-replay-de-sessão)).
- **Tópicos**: inspecionar, publicar (uma vez ou em taxa fixa) e chamar serviços.
- **Gráficos**: séries da telemetria ou de qualquer campo numérico de tópico (no painel principal).
- **Rede**: qualidade da conexão com histórico, túnel, token e uso de CPU/GPU/RTF.
- **Cenários**: arquivos, editor (formulário com mapa, formas e CSV/Excel, ou YAML), pré-visualização dos
  percursos, execução e estado por drone ([§20](#20-scenarios-and-target-paths)).
- **Vista Gazebo** (painel da direita, ao lado de 3D/Gráficos): câmera livre do Gazebo ([§22](#22-câmera-livre-do-gazebo)).

### Latência

Cada quadro carrega os instantes de captura e de codificação; o navegador mede **captura → tela**
e decompõe em codificação, rede + fila e decodificação (cabeçalho e aba Rede). Medido nesta máquina
(RX 6600, VA-API), navegador local: ~35 ms em 1080p H.264, ~15 ms em 30%.

O que evita o atraso acumulado do Foxglove: o navegador confirma cada quadro; com mais de
`max_inflight` quadros sem confirmação o servidor **descarta em vez de enfileirar** e pede um
keyframe; no modo adaptativo o bitrate segue a banda medida. Num link simulado de 3 Mbit/s com o
codificador a 8 Mbit/s: sem adaptação a latência fica limitada (~300 ms, poucos fps); com adaptação,
~21 fps com mediana de 21 ms. Sem visualizador conectado o streamer não assina a câmera nem codifica.

### Serviço

`./gcs.sh install-service` cria `~/.config/systemd/user/aero-gcs.service` (sobe no login;
`sudo loginctl enable-linger $USER` para subir sem login gráfico). Reiniciar a GCS não derruba a
simulação. `./gcs.sh status | logs | uninstall-service`.

## 20. Scenarios and target paths

A scenario (`scenarios/*.yaml`) describes one reproducible chase: **one target** (the pursued drone)
and its path, the **pursuer** start and when the chase begins, and the number of runs. Every field,
commented: [`scenarios/reference.yaml`](scenarios/reference.yaml); the files are listed in
[`scenarios/README.md`](scenarios/README.md).

```yaml
name: my_chase
duration: 60                  # [s] per run; 0 = until the target path ends
repetitions: 3
target_speed: 0               # [m/s] fixed speed along the path; 0 = the path's own speeds
target:
  start: [5, -2, 1]
  path:                       # legs in order (target.loops repeats the list; 0 = forever)
    - trajectory: {type: figure8, center: [22, 0, 7], size: [12, 6, 1], speed: 2.5}
      loops: 2
    - trajectory: {type: timed, points: [[0, 20, 0, 6], [4, 28, 6, 7], [9, 20, 0, 6]]}   # t,x,y,z (CSV)
    - hover: 2
    - behavior: flee          # reacts to the pursuer (flee | random | orbit)
pursuer:
  start: [0, 0, 5]
  detection: vision           # vision | omniscient
  start_after: 2              # [s] then the chase is enabled
parameters: {/pursuit_controller_node: {target_distance: 5}}
```

**Paths.** A `trajectory` leg is `waypoints` (constant speed, straight or `spline`, per-point
`speed`/`hold`), `timed` (a time per point, e.g. from a CSV: variable speed) or a shape (`circle`,
`polygon`, `figure8`, `lissajous`, `helix`, `random`). The reference is sampled by arc length
(`Trajectory::sample`), so a shape is flown at the same speed everywhere; the target follows it with
`v = v_ref + 1.2·(p_ref − p)` (saturated) through its velocity controller (acceleration ≤ 3 m/s²).
A leg that starts far from the drone is joined by a straight transit.

**Run** (`scenario_runner`, always loaded): `load` spawns the target if missing → `start` →
**STAGING** (target and pursuer fly to their starts; with `face_target` the camera is turned to the
target) → **RUNNING** (path from ground truth, chase enabled after `start_after`, optionally only once
the tracker has the target) → end by `duration`, path finished or `capture_distance`; repeated
`repetitions` times. With `acquisition: auto`, `target_acquisition` projects the target box into the
camera and starts the tracker by itself (`reacquire` restarts it if lost). Each run is a dataset
session (`datasets/<date>_<run_id>/`).

```bash
./run_scenario.sh figure8_basic                      # from scratch, headless
SCENARIO=chained_laps ./start_simulation.sh          # tmux session
ros2 run aero_pursuit scenario_tool preview scenarios/figure8_basic.yaml   # validate / preview
```

**In the GCS** (Scenarios tab): files, a form with an explanation per field (Scenario, Target,
Pursuer, Controller, Advanced) and the path as a list of legs. **✎ Edit on the map** opens a map
window seen from above with the height and speed over time below it: click to add points, click a
line to insert, drag to move, drag in the lower chart to change the height; or pick a **shape**
(generator with its parameters, convertible to points), or **import a CSV/Excel table** (t, x, y, z;
also pasted cells). The curve drawn is computed by the same C++ that flies it (`scenario_tool
sample`). **? Help** explains every field and shows the YAML reference; **YAML** edits the text.

## 21. Estudos em lote

Um estudo (`scenarios/studies/*.yaml`) combina cenários × variantes × repetições; variantes
sobrescrevem qualquer campo do cenário (controlador, detecção, duração, parâmetros...):

```yaml
name: controllers_vs_trajectories
repetitions: 3
scenarios: [../figure8_basic.yaml, ../tuning/s3_figure8.yaml]
variants:
  - {name: mpc_vision, parameters: {/pursuit_controller_node: {controller_type: mpc}}}
  - {name: mpc_omniscient, pursuer: {detection: omniscient}}
```

```bash
./ros2_ws/run_study.py scenarios/studies/controllers_vs_trajectories.yaml   # --dry-run, --gui, --keep-running
```

Inicia a simulação headless se preciso, executa tudo e grava em `datasets/studies/<estudo>_<data>/` os cenários combinados (para
reprodução) e `summary.csv` com uma linha por execução (RMSE, esforço, tempo por modo, IoU, fração no
alvo, trocas de identidade, caminho da sessão).

Sintonia em lote (opção A do TCC): cada avaliação roda os cenários de treino com os ganhos candidatos
e calcula o custo comum J pela `session_info.yaml`; Nelder–Mead em log θ. Resultado em
`datasets/tuning/<nome>_<data>/` (`evaluations.csv`, `best.yaml`). Especificações prontas: uma por
lei × fonte em `scenarios/tuning/tune_*.yaml`.

```bash
./ros2_ws/tune_study.py scenarios/tuning/tune_mpcpx4_omniscient.yaml --max-evals 15
./ros2_ws/identify_px4_lag.py datasets/<sessão> [...]     # τ_eq para mpc.lag.tau
```

## 22. Câmera livre do Gazebo

O `spectator_camera` insere o modelo `spectator_camera` (câmera 1280×720 sem colisão) e o move por
`set_pose`; a imagem vai à GCS por um segundo canal de vídeo (`video_streamer_spectator`, mesmo
pipeline de baixa latência). Só é renderizada com a vista **Gazebo** aberta (ponte lazy).

- **Livre**: arrastar = olhar, W/A/S/D = andar, Q/E = descer/subir, Shift = rápido, roda = velocidade.
- **Seguir / Orbitar / Olhar para** um drone: arrastar gira em torno dele, roda = distância.

Por tópico: `ros2 topic pub --once /spectator/command aero_pursuit/msg/SpectatorCommand "{mode: follow, target: X4, distance: 8.0, elevation: 0.3}"`.

## 24. Docker

Tudo (Gazebo, PX4 SITL, Micro-XRCE, OpenCV 4.9, aero_pursuit e GCS) num contêiner, sem instalar nada
além do Docker. Em Linux com `apt`, o próprio `aero.sh` prepara o host na primeira vez (pede a senha do
sudo): instala Docker Engine, Compose v2 e buildx se faltarem (`docker.io`/`docker-compose-v2`/
`docker-buildx` do Ubuntu, ou os plugins do repositório da Docker se o Docker veio de lá), liga o
serviço e põe o usuário no grupo `docker`. **Não rode com `sudo`**: os arquivos criados no repositório
ficariam do root. `AERO_SKIP_DOCKER_SETUP=1` pula essa preparação.

```bash
./docker/aero.sh image                  # constrói a imagem (uma vez; ~30-50 min)
./docker/aero.sh                        # GCS em http://localhost:8080 → aba Simulação/Cenários
./docker/aero.sh build                  # após editar C++/mensagens (incremental)
./docker/aero.sh scenario figure8_basic  # cenário headless
./docker/aero.sh study scenarios/studies/controllers_vs_trajectories.yaml
./docker/aero.sh test | shell
GUI=1 ./docker/aero.sh                  # janela do Gazebo no host (X11)
```

- **Imagem enxuta**: multi-estágio; OpenCV (só os módulos usados), PX4 (só `bin/`, `etc/`, plugins e
  modelos do SITL) e o agente XRCE são compilados em estágios descartados. A imagem final tem o ROS 2
  base + Gazebo + o compilador, para compilar o workspace montado.
- **Código por volume**: o repositório é montado em `/home/aero/aero_pursuit`; `build/`, `install/` e
  `log/` do workspace ficam em volumes nomeados (não se misturam com uma compilação nativa no host).
  Na primeira execução o workspace é compilado sozinho. Cache dos modelos do Fuel e configuração da GCS
  também em volumes. Datasets, logs e cenários ficam no próprio repositório do host.
- **GPU**: `/dev/dri` (AMD/Intel, Mesa: OGRE2 e VA-API). NVIDIA: com o `nvidia-container-toolkit`,
  o `aero.sh` acrescenta `docker/compose.nvidia.yaml` sozinho.
- **Rede**: `network_mode: host` e `ipc: host` (DDS, gz-transport e GCS sem mapear portas); use
  `ROS_DOMAIN_ID` diferente se houver outra simulação ROS na máquina.
- O contêiner roda com o UID/GID do usuário do host (arquivos gerados continuam seus).
