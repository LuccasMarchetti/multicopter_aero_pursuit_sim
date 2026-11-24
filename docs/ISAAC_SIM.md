# Perseguição no Isaac Sim

A mesma perseguição do Gazebo, agora no NVIDIA Isaac Sim 5.1 com o Pegasus Simulator 5.1.0 e a
PX4 v1.18 (mesmo commit do `docker/Dockerfile`). Os nós do perseguidor, o `scenario_runner`, os
cenários, o gravador e as métricas não mudam. O plano e as decisões estão em
`docs/PLANO_ISAAC_SIM.md`.

## Máquina usada nos testes (referência, não configuração)

- **Host do Isaac:** `bismark` (Ubuntu 22.04, 2× RTX A2000 12 GB, driver 550.163, Xeon w3-2435,
  128 GB). É uma máquina compartilhada do laboratório. Acesso por SSH com chave, pela Tailscale
  (`luccas@100.68.147.83`) ou pela ZeroTier (rede `Aerotech-TCC`, `luccas@10.121.234.239`).
- **Latência:** pela Tailscale o RTT fica em 17 a 60 ms. Pela ZeroTier o tráfego passa por relay
  (~240 ms), porque o bismark está na rede da USP. Em qualquer caso **toda a malha de controle
  roda no bismark** (Isaac, PX4, agente XRCE, perseguidor, GCS). Os outros computadores só abrem
  a GCS de lá no navegador (pelo lançador, [COMANDOS.md](COMANDOS.md)).
- **Isolamento:** `ROS_DOMAIN_ID=42`, com o mesmo perfil Fast DDS dos dois lados
  (`isaac/fastdds_shm.xml`). A descoberta é UDP só no loopback e os dados vão por memória
  compartilhada. Nada de DDS sai do bismark.

## Componentes

| Peça | Onde | O que é |
|---|---|---|
| Isaac Sim 5.1 | `~/isaacsim51` (bismark) | Instalação *workstation* (pacote oficial), copiada da instalação de `aerotech` para não alterar a dela |
| Pegasus 5.1.0 | `~/PegasusSimulator` | Instalado no Python do Isaac (`pip install -e`) |
| PX4 v1.18 | `~/PX4-Autopilot` | `px4_sitl_default`, airframe `gazebo-classic_iris` (HIL por MAVLink, lockstep) |
| Mundo | `isaac/aero_world.py` | App Isaac: x500 + câmera, alvos, `/clock`, `/tf`, mundo ROS (`/sim/*`) |
| ROS | imagem `aero-isaac` | `docker/Dockerfile.pursuer --target isaac`: aero_pursuit (`AERO_SIM=OFF`), agente XRCE 2.4.3, foxglove_bridge, GCS |
| Launch | `isaac_world.launch.xml` | Agente XRCE, TFs da câmera, scenario_runner (`world_backend:=ros`), perseguidor, gravador, streamer de vídeo da GCS |
| GCS | contêiner `aero-isaac-gcs` | A mesma GCS do Gazebo, com o backend Isaac (`AERO_GCS_SIM=isaac`) |
| Agente do host | `isaac/host_agent.py` | Executa `isaac/host.sh` a pedido da GCS (socket Unix), vive enquanto a GCS estiver ligada |

### Por que nativo + Docker

O Isaac Sim roda nativo (instalação *workstation*, um dos métodos oficiais). Em contêiner ele
precisaria do NVIDIA Container Toolkit, que não está instalado no bismark (o Docker só tem o
runtime `runc`). Instalar exigiria mexer no daemon Docker de uma máquina compartilhada, e não
traz ganho, porque a GPU e o driver são os mesmos nos dois casos.

O ROS fica separado. O Isaac usa as bibliotecas **ROS 2 Jazzy internas** do bridge (Python 3.11,
`~/isaac_env.sh`), sem nenhum ROS do sistema carregado. O pacote aero_pursuit (Jazzy) roda no
contêiner `aero-isaac`, porque o Ubuntu 22.04 não tem Jazzy nativo. A documentação do Isaac 5.1
indica Docker justamente para workspaces Jazzy no 22.04. Os dois lados usam o Fast DDS 2.14.

Problemas encontrados que motivaram essas escolhas:

- **Humble interno (Fast DDS 2.6) × Jazzy do contêiner (2.14):** a memória compartilhada é
  incompatível entre as versões, e por UDP as imagens cruas de 1080p acumularam até 122 GB no
  processo do Isaac, que acabou morto pelo OOM killer. Solução: Jazzy interno no Isaac.
- **`ROS_LOCALHOST_ONLY` no Humble × Jazzy:** os dois implementam o modo de formas diferentes e
  não se descobriam. Solução: o perfil XML explícito, igual nos dois lados.
- **Segmento SHM de 512 KB (padrão):** cada quadro (6 MB de RGB + 8 MB de profundidade) caía em
  fragmentos UDP e o rastreador recebia ~10 dos 31 quadros/s. Solução: segmentos de 64 MB.
  O rastreador passou a 30 Hz, como no Gazebo.
- **Proteções para a máquina compartilhada:** o Isaac roda com `MemoryMax=48G`
  (`systemd-run --user --scope`). Os buffers UDP do kernel foram aumentados para 8 MB
  (`/etc/sysctl.d/60-aero-dds.conf`).

## Modelo do perseguidor (x500 no Pegasus)

O Pegasus só traz o Iris. A geometria é a do Iris, com a dinâmica do x500 usado no Gazebo:

- 2,0 kg, inércia (0,0217; 0,0217; 0,040) kg·m²;
- empuxo `8,54858e-6·ω²`, torque de arrasto `0,016·empuxo`;
- ESC ω = 150 + 850·u rad/s (`SIM_GZ_EC_MIN/MAX`);
- arrasto linear (0,25; 0,25; 0) N/(m/s), que é o arrasto de rotor do Gazebo no pairado;
- parâmetros da PX4 do airframe 4001 (`CA_ROTOR*`, `MPC_THR_HOVER`) via `PX4_PARAM_*`, mais
  `NAV_DLL_ACT=0` (o mesmo de `launch_gz_and_drones.sh`).

Identificação com `identify_px4_lag.py` numa sessão no Isaac:

| Malha | Isaac | Gazebo |
|---|---|---|
| Velocidade (τ_eq) | 0,19 s (atraso 0) | 0,20 s |
| Yaw | τ 0,28 s, atraso 0 | τ 0,30 s, atraso 0,16 s |

A PX4 do Pegasus é lockstep com o Isaac. O RTF ficava um pouco abaixo de 1 (0,74 a 0,95) e,
depois das otimizações de "Desempenho", fica em 1,0. O
offset de timesync do uXRCE corre atrás dessa deriva e o fluxo offboard parecia velho (falha de
offboard logo após armar). Por isso o `offboard_control` ganhou `px4_receive_stamp`: com ele, os
timestamps vão em 0 e a PX4 carimba cada mensagem na chegada. Fica ligado só no launch do Isaac.

## Câmera e alvo

- Câmera fixa na montagem do x500_depth (`camera_link` em 0,12; 0,03; 0,242 e o sensor no
  offset do SDF). RGB 1920×1080 com HFOV 69° (fx = 1397,2). Os tópicos são os mesmos do
  Gazebo.
- **Profundidade:** por padrão sai do mesmo render da RGB (registrada, 1920×1080, 69°, 32FC1 em
  metros). Uma segunda câmera igual à OV7251 do Gazebo (640×480, 73°, `--depth separate`) derruba
  o RTF de ~1,0 para 0,53 nas duas A2000. Outra diferença é que a profundidade não é cortada em
  19,1 m.
- **Renderização:** a cada 8 passos de física (250 Hz), com câmeras a 31,25 Hz, nas duas GPUs
  (multi-GPU; com uma só o RTF cai para 0,36).
- **Alvos:** drones sem PX4, comandados pelo `scenario_runner` pelo mundo ROS
  (`/sim/spawn|remove|set_pose|twist|enable`, `/sim/poses`, `sim/ros_world.hpp`), **com física**:
  ver "Alvos com física" abaixo. A aparência é a **dos próprios modelos do
  Gazebo** (X4 e X3 do Fuel, X5 = x500 da PX4, tiltrotor da PX4), convertidos para USD por
  `isaac/import_gz_models.py`. Com a geometria procedural de reserva (`target_models.py`), o
  ViTTrack perdia o alvo quase o tempo todo.
- **Cenário:** o ambiente é o "Flat Plane" do Pegasus (piso cinza, céu cinza), não o `lawn` do
  Gazebo. Mudam o fundo e a iluminação; as distâncias e trajetórias são as mesmas.

## Uso

No computador da GPU, com o repositório clonado e a GCS instalada (`./aero install`,
[COMANDOS.md](COMANDOS.md)):

```bash
aero isaac setup                 # uma vez: Isaac 5.1, Pegasus, PX4, ~/isaac_env.sh (pede sudo)
aero isaac build                 # imagem aero-isaac (lado ROS; após mudar o código C++/ROS)
aero sim isaac                   # GCS deste computador no Isaac (ou o seletor da aba Simulação)
aero sim start                   # mundo + ROS; o x500 decola sozinho a 5 m
aero study scenarios/studies/isaac_e3.yaml   # estudo (run_study.py --external-sim)
aero isaac px4 commander status  # console do PX4
aero isaac status | aero logs world|ros|px4|agent|study | aero isaac down
```

O lado do host fica em `isaac/host.sh` (mundo, contêiner ROS, build, PX4, agente). A GCS chama esse
script direto (modo nativo) ou pelo agente do host (modo docker). Cenários avulsos rodam pela aba
Cenários da GCS.

Para converter os modelos dos alvos (uma vez, no computador do Isaac; os do Fuel são baixados, x5 e
tiltrotor vêm dos modelos do pacote; saída em `isaac/assets/`, fora do git):

```bash
mkdir -p ~/.cache/aero/gz_assets && cd ~/.cache/aero/gz_assets
curl -sfL -o x4.zip "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X4%20UAV%20Config%201/15/X4%20UAV%20Config%201.zip" && unzip -q -o x4.zip -d x4
curl -sfL -o x3.zip "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/4/X3%20UAV.zip" && unzip -q -o x3.zip -d x3
A=$PWD; G=~/aero_pursuit/ros2_ws/src/aero_pursuit/gz_models; cd ~/aero_pursuit
isaac/isaac_python.sh isaac/import_gz_models.py x4=$A/x4 x3=$A/x3 x5=$G/x5 tiltrotor=$G/tiltrotor
```

O importador segue os `<include>` (`model://x500` → `x500_base`, na pose do *merge*, como o
Gazebo) e procura `model://<modelo>/...` nos modelos gz da PX4, em `gz_models` do repositório e
em `GZ_SIM_RESOURCE_PATH`. Ele tem dois tratamentos próprios:

- **COLLADA do MeshLab ou só com geometria** (asa X8 do tiltrotor, hélices do X3): o conversor do
  Kit dá segfault nesses arquivos, então o importador lê os triângulos e as cores difusas por
  conta própria.
- **Materiais:** desliga as texturas de opacidade (o quadro de fibra de carbono do x500 ficava
  transparente) e usa a cor média da textura como cor constante. Malhas sem material (STL)
  recebem a cor do SDF (`diffuse`/`ambient` ou `Gazebo/DarkGrey`).

Para conferir: `isaac/isaac_python.sh isaac/preview_model.py isaac/assets/<chave>.usd --out /tmp/<chave>`
(frente, lado e topo). `docs/isaac_alvos.png` mostra X3, X4, X5 e tiltrotor, nessa ordem.

As métricas saem com o script de sempre:
`latex/monografia/scripts/metricas_ensaio.py datasets/isaac/<sessão>`.

## Alvos com física

Até 17/11 os alvos eram cinemáticos: a posição era integrada a partir da velocidade comandada, com
uma resposta de primeira ordem, sem gravidade e sem inclinação. Por isso pareciam "parados no ar".
Desligados, continuavam flutuando, e não inclinavam para acelerar.

Agora cada alvo é um corpo rígido do PhysX (`DynamicDrone` em `isaac/aero_world.py`), com:

- **massa e inércia do SDF** do modelo do Gazebo e gravidade (`target_models.DYNAMICS`);
- **colisor** de caixa invisível do tamanho do visual, com o centro de massa no centro do visual
  (o frame do modelo, o da TF, continua o do Gazebo);
- **o mesmo controlador do Gazebo**: o `LeeVelocityController` do `MulticopterVelocityControl`,
  com os ganhos do SDF de cada modelo (`velocityGain`, `attitudeGain`, `angularRateGain`, limites
  de aceleração e velocidade). O comando de velocidade no referencial de rumo vira empuxo coletivo
  no eixo z do corpo e torque. Assim, o drone inclina para acelerar (até ~21° com o limite de
  3 m/s²), nivela em cruzeiro e inclina para trás para frear;
- **desligado** (`/sim/enable "<nome> 0"`), sem empuxo: cai e pousa, como no Gazebo.

`--kinematic-targets` volta ao modelo antigo. Os quatro alvos custam ~0,7 ms por passo de física.

**Drones de base.** Como no Gazebo (`launch_gz_and_drones.sh`), o mundo sobe com X3, X4,
Tiltrotor e X5 a 1 m (os mesmos `base_drones` do `scenario_runner`), habilitados e pairando,
prontos para o joystick. `--base-drones` sem valor sobe sem eles.

## Teleoperação, câmera livre e gimbal

- **Joystick/teclado da GCS.** O `drone_joy_teleop` roda no launch do Isaac com
  `target_interface: ros`: o alvo selecionado recebe `TwistStamped` em `/sim/twist` e um
  `/sim/enable` ao ser selecionado. O perseguidor continua pelo `offboard_control`, como no Gazebo.
- **Câmera livre (vista "Isaac" da GCS).** O mesmo `spectator_camera` do Gazebo, agora sobre a
  interface `sim::World` (`world_backend: ros`). Ele "insere" o modelo `spectator_camera` pelo
  mundo ROS e o move por `/sim/set_pose`. O `aero_world.py` trata esse modelo como uma câmera
  1280×720 com HFOV 1,15 rad (a do SDF do Gazebo), publicada em `/spectator/image`. Os modos são
  os da GCS: livre, seguir, orbitar e olhar para um drone.
  - **Custo:** um segundo produto de render custa ~3 ms por quadro de câmera, qualquer que seja a
    resolução (640×360 custou o mesmo que 1280×720). O RTF cai de 1,00 para 0,92–0,94. Por isso
    a câmera só renderiza enquanto alguém assina `/spectator/image`. Ligar e desligar o render a
    cada quadro, para reduzir o FPS, custa ~23 ms por troca e derrubou o RTF para 0,52, então
    não é feito.
  - **Slider de FPS** na barra da vista (0 a 30): limita o vídeo extraído (codificação e banda).
    Em 0 a câmera desliga: o streamer cancela a assinatura e o Isaac para de renderizar, e o RTF
    volta a 1.
- **Gimbal: modo `manual`.** Novo modo do `gimbal_controller` (Gazebo e Isaac): o operador
  comanda taxas por `/gimbal/manual_rate` (`geometry_msgs/Vector3Stamped`, x = yaw, y = pitch,
  rad/s; o comando zera se ninguém enviar por 0,5 s). O azimute é relativo à proa do corpo e a
  elevação é absoluta, com o horizonte nivelado. O serviço `/gimbal_controller/center` volta à
  posição padrão (frente, `neutral_pitch_deg`). Na GCS, aba Perseguição → Gimbal: um **pad**
  (arrastar ou setas) e o botão **⌂ Posição padrão**. O primeiro movimento passa o gimbal para
  `manual` sem lembrar o valor, e "↩ Voltar a <modo>" restaura o modo anterior.

## Cenários de perseguição

Os cenários são os mesmos nos dois simuladores: não há cenário "do Gazebo" ou "do Isaac". A GCS
lista `scenarios/` e as subpastas (menos `studies/`); todos os campos valem nos dois.

Os cenários `helix_climb`, `lissajous_fast`, `takeoff_chase` e `random_evasive` (o antigo
`mixed_decoys`, com distratores, foi removido junto com os enxames) estão em `scenarios/` e no estudo `scenarios/studies/cenarios_novos.yaml`, que roda
nos dois simuladores. As antigas cópias `scenarios/isaac/*` (iguais às de `tuning/`, renomeadas)
foram removidas.

Primeira execução (17/11, uma repetição, IBVS-MPC *perception-aware* com visão, estudo
`cenarios_novos`; `latex/monografia/dados/cenarios_novos_{gazebo,isaac}.csv`). Todos rodam do começo
ao fim nos dois simuladores. São cenários difíceis de propósito, e com n = 1 a diferença entre
simuladores ainda não é conclusiva (J_P "—": sem trecho em regime).

| Cenário | J_P Gazebo | J_P Isaac | TRACKING Gz | TRACKING Isaac | FOV Gz | FOV Isaac |
|---|---|---|---|---|---|---|
| helix_climb | 124,60 | 14,44 | 0,69 | 0,93 | 0,75 | 0,99 |
| lissajous_fast | 77,83 | 328,50 | 0,73 | 0,28 | 0,91 | 0,53 |
| takeoff_chase | 3,91 | 54,82 | 1,00 | 0,94 | 1,00 | 0,96 |
| random_evasive | — | 82,88 | 0,39 | 0,75 | 0,86 | 0,90 |
| mixed_decoys | 116,00 | 80,93 | 0,49 | 0,47 | 0,58 | 0,87 |

## Cenografia (árvores)

Ambiente mais rico, sem obstáculo: árvores em volta da área de voo e ao fundo (o perseguidor olha
para +x), para mostrar o rastreamento com fundo poluído. Usa só o que já existia: os modelos
**Oak Tree** e **Pine Tree** do Fuel, os mesmos do mundo `forest` da PX4.

- **Um arquivo para os dois simuladores:** `ros2_ws/src/aero_pursuit/config/scenery/<nome>.json`
  (modelo, x, y e rumo de cada árvore; JSON porque o Python do Isaac não tem PyYAML). As árvores
  ficam fora da área de voo dos cenários (x de −8 a 47 m, |y| < 19 m), com no mínimo 6 m entre
  elas.
  - `forest_edge`: 58 árvores (fundo, laterais e atrás da partida);
  - `forest_sparse`: 25 delas, nas mesmas posições.
- **Gazebo:** `AERO_SCENERY=<nome>` (ou a GCS) faz o `launch_gz_and_drones.sh` gerar, com
  `ros2_ws/tools/scenery_world.py`, uma cópia do `lawn.sdf` com um `<include>` estático por
  árvore. O mundo continua `lawn`, então pontes, TFs e frames não mudam. Na primeira vez o Gazebo
  baixa os dois modelos do Fuel.
- **Isaac Sim:** `--scenery <nome>` (ou a GCS) referencia `isaac/assets/oak_tree.usd` e
  `pine_tree.usd` nas mesmas poses, só como visual (sem colisor). Os assets são importados uma vez:

  ```bash
  mkdir -p ~/.cache/aero/gz_assets && cd ~/.cache/aero/gz_assets
  for m in "Oak Tree:oak_tree" "Pine Tree:pine_tree"; do n="${m%%:*}"; k="${m##*:}"
    curl -sfL -o $k.zip "https://fuel.gazebosim.org/1.0/OpenRobotics/models/${n// /%20}.zip" && unzip -q -o $k.zip -d $k; done
  A=$PWD; cd ~/aero_pursuit && isaac/isaac_python.sh isaac/import_gz_models.py oak_tree=$A/oak_tree pine_tree=$A/pine_tree
  ```

  Os COLLADA das árvores não têm textura: no Gazebo, a cor vem do script OGRE de cada submesh
  (`OakTree/Branch` com `alpha_rejection`). O importador passou a ler esses scripts e a aplicar a
  textura e o recorte por alfa ao material de cada parte (folhas recortadas, casca texturizada).
- **GCS:** aba Simulação → Cenário visual → Cenografia (vale no próximo início, nos dois
  simuladores).
- **Custo no Isaac:** o recorte por alfa das folhas é caro no traçado de raios. Medido com
  `--profile`:

  | Cenografia | render por quadro | RTF |
  |---|---|---|
  | nenhuma | 12,6 ms | 1,00 |
  | `forest_edge` com sombra das árvores | 27,4 ms | 0,71 |
  | `forest_edge` sem sombra das árvores (padrão) | 19,7 ms | 0,85 |
  | `forest_sparse` sem sombra das árvores | 17,9 ms | 0,90 |

  Por isso as árvores não projetam sombra no Isaac (`primvars:doNotCastShadows`;
  `--scenery-shadows` volta a projetar). No Gazebo, o efeito no fator de tempo real ainda não foi
  medido.

## O que foi adaptado no código

- `sim/world.hpp`: interface `sim::World`. `GzWorld` passou a implementá-la, e `RosWorld` é a
  implementação nova sobre tópicos ROS com tipos padrão (o rclpy do Isaac não conhece as
  mensagens do aero_pursuit).
- `scenario_runner`: novo parâmetro `world_backend: gz | ros`. Passou a ser compilado também
  com `AERO_SIM=OFF`, e no modo ros o SDF do catálogo é opcional. Novo `publish_model_tf`
  (desligado no Isaac, que já publica a `/tf` de todos os modelos).
- `offboard_control`: parâmetro `px4_receive_stamp` (descrito acima).
- `run_study.py`: opção `--external-sim`, para rodar com um simulador já aberto, sem
  `sim_manager.sh`.
- `docker/Dockerfile.pursuer`: novo alvo `isaac` (com GStreamer e o streamer de vídeo da GCS).
  O `pursuer.sh` agora pede `--target runtime` explicitamente.
- `CMakeLists.txt`: opção `AERO_GCS_VIDEO` (padrão igual a `AERO_SIM`) para o streamer de vídeo.
- GCS: `IsaacSimControl` em `sim_control.py` (escolhido por `AERO_GCS_SIM=isaac`) e
  `host_client.py`.
- `spectator_camera`: parâmetro `world_backend: gz | ros`, compilado também sem `AERO_SIM`.
- `drone_joy_teleop`: parâmetro `target_interface: gz | ros`.
- `gimbal_controller`: modo `manual`, tópico `manual_rate_topic` e serviço `~/center` (vale no
  Gazebo também).
- `isaac/aero_world.py`: alvos com física (`DynamicDrone`), drones de base, câmera livre
  (`SpectatorCamera`), cenografia (`--scenery`) e as otimizações de "Desempenho".
- `isaac/import_gz_models.py`: materiais por submesh a partir dos scripts OGRE (árvores).
- `launch_gz_and_drones.sh` + `tools/scenery_world.py`: cenografia no Gazebo (`AERO_SCENERY`).
- GCS: lista de cenários com subpastas e avisos
  (`scenarios.py`), pad do gimbal (`gimbal_pad.js`), gravação (`recorder.js`), slider de FPS da
  câmera livre (`spectator.js`).
- No Gazebo nada muda com os padrões novos (`world_backend: gz`, `publish_model_tf: true`,
  `px4_receive_stamp: false`, `target_interface: gz`).

## GCS

A GCS do computador do Isaac é a mesma de qualquer computador (`./aero install`, [COMANDOS.md](COMANDOS.md)).
Sem ROS no host, ela roda no modo docker: contêiner `aero-gcs` (imagem `aero-pursuit`, com o Gazebo),
ligado ao Isaac do host pelo agente. Fica ligada entre simulações e sobe sozinha depois de um
reboot (`@reboot` no crontab).

- **Acesso:** pelo lançador, ou `https://100.68.147.83:8443` (Tailscale, certificado
  autoassinado, pede o token: `aero gcs token` lá).
- **Simulação:** a aba Simulação inicia, para e reinicia o Isaac e o contêiner `aero-isaac` com o
  mesmo formulário do Gazebo. Os campos sem efeito no Isaac (mundo, Gazebo, treinamento,
  joystick USB) ficam ocultos. Com câmera `gimbal`, o mundo sobe com `--camera gimbal`.
  "Compilar" constrói a imagem `aero-isaac` a partir do repositório (`host.sh build`).
  Os logs `world` e `ros` são os arquivos de `~/.local/state/aero/isaac/logs`. O console do PX4 usa os clientes
  `px4-<módulo>` da instância 0, e a saída aparece no log `gcs`.
- **Cenários:** os mesmos do Gazebo (ver "Cenários de perseguição"). Rodar um cenário com a
  simulação no ar usa o serviço do `scenario_runner`, como no Gazebo.
- **Estudos:** rodam onde a GCS roda (`run_study.py --external-sim`, mesmo domínio
  DDS). A simulação precisa estar no ar.
- **Vídeo:** o `video_streamer` está no launch do Isaac (`gcs_stream:=true`) e fica ocioso sem
  visualizador. O H.264 sai do NVENC: sem o NVIDIA Container Toolkit, `docker/nvidia_host.sh`
  monta no contêiner os dispositivos `/dev/nvidia*` e as bibliotecas do driver do host, e o
  streamer usa `nvh264enc` quando não há VA-API (x264 na CPU só sem GPU). Para isso, a imagem compila o streamer com `-DAERO_GCS_VIDEO=ON`, opção
  nova do CMake que antes vinha junto com `AERO_SIM`. O socket de vídeo fica em
  `~/.local/state/aero/isaac/run`, montado nos dois contêineres (`XDG_RUNTIME_DIR`).
- **Ambiente do Isaac:** campo "Ambiente do Isaac Sim" (`--env`; `Flat_Plane` = chão de ladrilhos cinza, o padrão; `Rough_Plane`, `Slope_Plane`, `Stairs_Plane`, `Warehouse`...).
- **Sem recursos do Gazebo:** vento, neblina e o plugin aerodinâmico (aba Física e clima) não
  existem no Isaac.
- **Parâmetros lembrados:** com "lembrar parâmetros" ligado, a GCS do Isaac reaplica os valores
  salvos em `~/.config/aero_gcs/settings.json` sempre que os nós sobem (por exemplo `output`,
  `target_distance` e `target_speed_scale`). Os estudos fixam os seus próprios valores.

### Gazebo e Isaac na mesma GCS

Toda GCS escolhe o simulador na aba Simulação → "Simulador desta GCS" (`settings['backend']`,
`aero_pursuit_gcs/backends.py`; ou `aero sim gazebo|isaac`). Trocar reinicia o processo da GCS
(`execv`), porque o nó ROS nasce uma vez por processo e o Isaac usa outro domínio DDS e perfil
(`ROS_DOMAIN_ID=42`, `isaac/fastdds_shm.xml`); a volta ao Gazebo restaura o domínio de partida. A
troca é recusada com a simulação rodando. O Isaac só aparece habilitado com GPU NVIDIA RTX e o
Isaac instalado (`aero isaac setup`), ou, no modo docker, com o agente do host no ar. Sem o agente,
a GCS nativa chama `isaac/host.sh` direto.

### Gravação da sessão

O botão **⏺ Gravar** no topo da GCS grava a própria aba em vídeo (câmera, 3D, HUD e painéis) pelo
navegador (`getDisplayMedia` + `MediaRecorder`) e baixa um MP4 (H.264, no Chrome) ou WebM ao
parar, com nome `aero_<simulador>_<data>_<hora>`. Não usa o servidor, então funciona igual nos
dois simuladores. O navegador pergunta o que compartilhar (a aba atual vem primeiro) e exige
contexto seguro (`https://<ip>:8443` ou `localhost`). Os dados da mesma execução continuam no
`datasets/<sessão>` do `session_recorder`, para métricas e reprodução numérica.

### Por que um agente no host

No modo docker, a GCS roda em contêiner, mas o Isaac roda nativo no host (sem o NVIDIA
Container Toolkit). Uma chave SSH para `localhost` daria acesso persistente a uma máquina
compartilhada. O agente (`host_agent.py`, Python do sistema) só existe enquanto a GCS está ligada
(`aero gcs start`/`stop`). Ele escuta em `~/.local/state/aero/isaac/run/host.sock`, com permissão
600, e executa somente `host.sh up|world|ros|down|status|build|px4`, com argumentos em palavras
simples, sem shell. O contêiner da GCS usa `--pid host` para que o painel de processos veja o
Isaac e a PX4. O painel só lista processos do próprio usuário, já que outros usuários podem rodar a
própria PX4. O código da GCS vem do repositório montado no contêiner: mudanças valem com
`aero gcs restart`, sem recompilar a imagem.

## Desempenho

Medido com `--profile` (tempo de parede por fase do laço) e `--cprofile N` (cProfile de N passos
a partir de t = 20 s). Os valores são em ms por passo de física; o render conta por quadro (1 a
cada 8 passos). Para o RTF chegar a 1, cada bloco de 8 passos precisa caber em 32 ms.

| | física | render | ROS | RTF livre | RTF com cenário de visão |
|---|---|---|---|---|---|
| Antes | 2,46 | 12,6 | 0,73 | 0,89 | 0,74 a 0,79 |
| Depois | 1,63 | 13,4 | 0,17 | ~1,15 | 1,00 (com folga de ~0,5 ms/passo) |

O que mudou em `aero_world.py`:

- **Executor ROS numa thread.** O `rclpy.spin_once` a cada passo montava o *wait set* (18
  entidades) 250 vezes por segundo, 0,65 ms por passo, mesmo sem mensagem nenhuma. Agora as
  callbacks só enfileiram, e o laço aplica os comandos na thread principal (o USD não é
  *thread-safe*). Os sinais (SIGINT/SIGTERM) passaram a ser tratados pelo próprio mundo, que sai
  do laço sem publicar num contexto ROS já encerrado.
- **Estado do veículo sem USD** (`fast_update_state`). O `update_state` do Pegasus lia a atitude
  do USD e convertia tipos do carb com `np.array`, a ~36 µs cada. Agora a pose inteira vem do
  `dynamic_control`, e caiu de 0,5 para 0,14 ms. A atitude é conferida contra a do USD com
  `--usd-physics` (`q_err` no relatório; ficou abaixo de 1e-6). `--pegasus-state` volta ao
  original.
- **Física no Fabric** (`omni.physx.fabric`). O PhysX deixa de escrever as poses no USD (e de
  acordar os *listeners* do Kit) a cada passo: −0,7 ms/passo. A câmera fixa e o gimbal seguem o
  corpo normalmente (rastreamento de 79 % e 83 %). `--usd-physics` volta ao modo antigo.
- **Alvos e gimbal escrevem no USD só antes de um render**, e não 250 vezes por segundo.

Sem efeito medido: número de threads do PhysX (0, 2 ou 8) e o CCD da cena. O perfil da CPU do
bismark está bom: o governor é `powersave`, mas com EPP `performance` os núcleos sobem a ~4 GHz.

O que sobra é o render (1080p RGB + profundidade, ~13 ms por quadro, nas duas GPUs) e o Python
do Pegasus (sensores e MAVLink, ~0,8 ms/passo). Com o vídeo da GCS (x264 na CPU) aberto, o RTF
fica em 0,98 a 1,00.

## Gimbal

Campo "Câmera do perseguidor" = gimbal na GCS (ou `AERO_CAMERA_MOUNT=gimbal aero isaac up --camera gimbal`). Três juntas cinemáticas (yaw, roll,
pitch, limites do SDF do `x500_gimbal_depth`), que seguem `/gimbal/joint_command` com primeira
ordem de 30 ms, no lugar do laço de posição rígido do Gazebo. Elas publicam
`/gimbal/joint_states` a 125 Hz. A câmera gira como `Rz(yaw)·Rx(roll)·Ry(−pitch)` em torno do
`gimbal_mount`, a mesma rotação que o `gimbal_controller` (`joint_interface:=ros`) publica na TF.
A inércia da câmera continua no controlador, como no Gazebo.

## Resultados (17/11/2026, três repetições, RTF 1)

`scenarios/studies/isaac_e3.yaml` e `isaac_gimbal.yaml` com `--repetitions 3`, já com as
otimizações de "Desempenho" (RTF de 0,97 a 1,03 em todas as 54 execuções). Métricas pelo
`metricas_ensaio.py` (`latex/monografia/dados/isaac_e3_r3.csv`, `isaac_gimbal_r3.csv`; a rodada
anterior, com uma repetição e RTF ~0,8, está em `isaac_e3.csv` e `isaac_gimbal.csv`), com os
mesmos parâmetros do E3 do Gazebo (`params_sha1` d437d0975220). Isaac: média ± desvio-padrão
amostral (n = 3). Gazebo: E3 (onisciente), `prelim_ibvs_mpc_vision_full` (visão, média de 3) e
`ibvs_mpc_gimbal` (faixa).

| Cenário / variante | J_P Gazebo | J_P Isaac | e_d Gz | e_d Isaac | TRACKING Gz | TRACKING Isaac |
|---|---|---|---|---|---|---|
| s3 mpc_los | 19,24 | 18,67 ± 0,09 | 0,60 | 0,46 ± 0,01 | 1,00 | 1,00 ± 0,00 |
| s4 mpc_los | 4,96 | 2,84 ± 0,09 | 0,61 | 0,33 ± 0,05 | 1,00 | 1,00 ± 0,00 |
| s3 mpc_center | 5,00 | 3,58 ± 0,04 | 0,62 | 0,48 ± 0,00 | 1,00 | 1,00 ± 0,00 |
| s4 mpc_center | 5,38 | 2,76 ± 0,01 | 0,77 | 0,41 ± 0,00 | 1,00 | 1,00 ± 0,00 |
| s3 ibvs | 1,75 | 1,72 ± 0,15 | 0,56 | 0,55 ± 0,02 | 1,00 | 1,00 ± 0,00 |
| s4 ibvs | 1,74 | 0,85 ± 0,01 | 0,40 | 0,29 ± 0,00 | 1,00 | 1,00 ± 0,00 |
| s3 ibvs_classic | 12,16 | 7,67 ± 0,14 | 1,68 | 1,24 ± 0,02 | 1,00 | 1,00 ± 0,00 |
| s4 ibvs_classic | 21,16 | 7,39 ± 0,11 | 2,26 | 0,94 ± 0,02 | 1,00 | 1,00 ± 0,00 |
| s3 mpc_center_vision | 12,07 | 17,30 ± 0,36 | 1,35 | 1,84 ± 0,03 | 0,84 | 0,81 ± 0,01 |
| s4 mpc_center_vision | 3,74 | 5,39 ± 0,43 | 0,53 | 0,64 ± 0,03 | 1,00 | 0,97 ± 0,01 |
| s3 ibvs_vision | 4,68 | 4,91 ± 0,29 | 0,80 | 0,89 ± 0,05 | 0,84 | 0,81 ± 0,02 |
| s4 ibvs_vision | 2,73 | 2,22 ± 0,03 | 0,57 | 0,48 ± 0,00 | 1,00 | 0,96 ± 0,01 |
| s3 ibvs_classic_vision | — | 107,31 ± 144,36 | — | 4,01 ± 3,03 | — | 0,56 ± 0,44 |
| s4 ibvs_classic_vision | — | 16,88 ± 1,85 | — | 1,19 ± 0,05 | — | 0,92 ± 0,06 |
| s3 ibvs_level_gimbal | 0,69–0,79 | 0,71 ± 0,02 | 0,31–0,32 | 0,33 ± 0,00 | 1,00 | 1,00 ± 0,00 |
| s4 ibvs_level_gimbal | — | 0,62 ± 0,00 | — | 0,28 ± 0,00 | — | 1,00 ± 0,00 |
| s3 mpc_center_gimbal | 5,5–6,0 | 2,79 ± 0,05 | 0,87–0,89 | 0,40 ± 0,00 | 1,00 | 1,00 ± 0,00 |
| s4 mpc_center_gimbal | — | 2,29 ± 0,65 | — | 0,44 ± 0,18 | — | 1,00 ± 0,00 |

- **Reprodutibilidade:** os casos oniscientes têm desvio de até ~9 % da média, e a maioria das
  médias fica a até ~10 % da rodada de uma repetição feita com RTF ~0,8. As exceções são os casos
  de maior dispersão: `s4 ibvs_classic_vision` (16,9 contra 12,1) e `s4 mpc_center_gimbal`
  (2,3 contra 3,1, desvio de 28 %). A PX4 e a física estão em *lockstep*, então o RTF não altera a
  dinâmica. O que muda é o tempo de processamento do rastreador e do controle em tempo simulado,
  que com RTF < 1 era mais folgado. Não houve mudança sistemática nas leis com rastreamento
  estável.
- **Ordem das leis:** é a mesma nos dois simuladores. O IBVS *perception-aware* é o melhor, o
  clássico e o `los` são os piores. A cadeia de visão rastreia na mesma proporção (s3 ~0,81 a
  0,84; s4 ~0,96 a 1,00).
- **IBVS clássico com visão no s3:** uma das três execuções perdeu o alvo aos 13,8 s e não o
  recuperou (TRACKING 0,06, e_d 7,5 m, J_P 274). As outras duas ficaram em J_P ~24. A partida foi
  igual nas três, então é a lei: sem restrição de campo de visão, o alvo sai da imagem na curva.
  É a dispersão que uma repetição só não mostrava.
- **Diferenças esperadas** entre os simuladores: a dinâmica (arrasto linear aproximado, alvo
  cinemático), a renderização e o cenário. No Isaac o erro de distância tende a ser menor nos
  casos oniscientes.
