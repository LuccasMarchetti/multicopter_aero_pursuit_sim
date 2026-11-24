# Pendências (handoff entre sessões do Claude Code)

Última atualização: 2026-11-18 (`feat/gcs-launcher`, no GitHub, sem merge na `main`). Seções antigas mantidas como histórico.

## GCSs independentes + lançador (18/11/2026, branch `feat/gcs-launcher`)

Feito: mesmo `./aero install` e mesmos comandos em todo PC (docs/COMANDOS.md); Gazebo/Isaac escolhido
dentro de cada GCS; `launcher/aero_launcher.py` no PC de acesso. Removidos `isaac/isaac.sh`,
`isaac/aero_host`, peers e a tela inicial `/`. x99 = modo nativo; bismark = modo docker (`aero-gcs`,
imagem `aero-pursuit`) + agente do host, clone próprio com deploy key só de leitura (alias
`github-aero`). Testado no bismark: Isaac pela GCS nova com RTF 1,0; Gazebo na GPU (driver do host
montado no contêiner, `docker/nvidia_host.sh`) com RTF 1,0; vídeo da GCS por NVENC nos dois.

Falta:
1. Merge de `feat/gcs-launcher` na `main` quando o usuário pedir.
2. Apagar `~/aero_isaac` e `~/scenarios_backup_20261118` no bismark (não usados mais).

## Próxima sessão (a partir de 18/11/2026)

1. **Replay no simulador para o Isaac Sim**: hoje só o Gazebo reconstrói a sessão (nó
   `world_replay`, manual §7); no Isaac há só "só a gravação". Falta: no `isaac/aero_world.py`,
   aceitar `/sim/set_pose` também para o perseguidor do Pegasus (hoje só alvos e câmera livre) e
   parar o PX4 de lá no modo replay (equivalente ao `px4 shutdown` + mundo sem gravidade do Gazebo);
   depois liberar o botão "No simulador" para o backend isaac (`sim_ops.replay_start`).
2. **Perseguidor remoto no Isaac**: modo remoto no `isaac_world.launch.xml` e rede DDS comum entre a
   Pi (LAN) e o bismark (Tailscale).
3. **Gazebo + Isaac no mesmo PC**: testar o seletor de simulador num computador com RTX e Gazebo.
4. **Túnel Cloudflare**: ligar "Iniciar o túnel junto com a GCS" nas duas GCS, se quiser acesso
   público permanente.

## Próxima sessão (pedido de 17/11/2026, Isaac Sim + GCS) — branch `feat/isaac-gcs`

Feito em 17/11 (commits locais, sem push; detalhes em `docs/ISAAC_SIM.md`): GCS no Isaac (agente do
host), RTF 1,0, alvos com física, teleop, câmera livre + slider de FPS, gimbal `manual` + pad,
tela inicial `/` com as GCS registradas, gravação da aba (MP4), cenários comuns aos dois
simuladores (+5 novos), cenografia com árvores (`config/scenery/*.json`), seletor de ambiente do
Isaac (`isaac_env`: Flat_Plane, o padrão de ladrilhos cinza, continua o padrão; Rough/Slope/Stairs
Plane, Warehouse...), HUD em inglês (`pursuer`/`target`), `./ros2_ws/gcs.sh access` (tokens e links).

Falta (em ordem sugerida):

1. ~~**Controlador externo na GCS**~~ feito em 17/11 no Gazebo (`pursuer_host`, `pursuer_remote`,
   `pursuer_max_rtt_ms`; `aero_pursuit_gcs/remote_pursuer.py`; ver `docs/RASPBERRY.md`). Testado com a
   Pi: tópicos remotos a 17 Hz (JPEG) e offboard a 20 Hz; a parada por latência funcionou. Falta:
   Isaac (modo remoto no `isaac_world.launch.xml` + rede DDS comum entre Pi e o PC do Isaac).
2. ~~**Gazebo e Isaac no mesmo PC, escolhidos na GCS**~~ feito em 17/11 (`backends.py`, seletor na
   aba Simulação, `AERO_ISAAC_HOST=localhost` no `isaac.sh`; ver `docs/ISAAC_SIM.md`). Falta testar
   num PC com RTX que tenha Gazebo também (o bismark não tem Gazebo/PX4-gz; este PC não tem NVIDIA)
   e o contêiner oficial `nvcr.io/nvidia/isaac-sim` como alternativa à instalação nativa.
3. **Cloudflare para as duas GCS**: código pronto; **autostart da GCS do Isaac instalado** em 17/11
   (`@reboot` no crontab do bismark). O túnel público continua desligado (aba Rede → "Iniciar o
   túnel junto com a GCS" liga em cada GCS; não abri sem pedido).
4. ~~**Script único de acesso/instalação**~~ feito em 17/11: `./aero` (install, view, sim
   gazebo|isaac|start|stop|status, access, gcs/isaac/pursuer). `aero install` não foi rodado aqui
   (instala o serviço do usuário: fica a seu critério). `gcs.sh` agora lê a porta do settings.json.
5. ~~**Replay de sessão**~~ (17/11, versão final): **no simulador** (Gazebo sem perseguidor, sem
   PX4 e sem gravidade; `world_replay` teleporta os modelos pelas poses do bag e move o gimbal;
   vídeo/câmera livre renderizados de novo, exato por construção) ou **só a gravação** (rosbag,
   também no Isaac). A re-simulação pela dinâmica foi descartada (diverge: nós fora do passo da
   física). Falta: modo no simulador para o Isaac (teleporte do perseguidor do Pegasus no
   `aero_world.py` + parar o PX4 de lá).
6. ~~**HUD aeronáutico**~~ feito em 17/11 (botão HUD da câmera, `video.js drawAeroHud`). **Removido**
   a pedido (17/11): a câmera volta a ter só o HUD de distâncias/velocidades de antes.
7. **Arrumação** (17/11): CLI `./aero` (+ `aero study`), estudos em cascata, manual atualizado, x99
   instalado (`aero install`: serviço `aero-gcs`, linger já ativo), bismark atualizado (imagem
   aero-isaac nova, GCS reiniciada, autostart). RTF do Gazebo: 0,76 sem cenografia, 0,74 com
   `forest_edge`. Falta decidir: `SKILL.md` na raiz (cópia da skill escrita-humanizada) e `source/`
   (PDFs do branch `arquivos-projeto`). A simulação do Isaac que estava aberta no bismark só pega o
   código novo (replay no simulador, staging estrito) quando for reiniciada.
8b. **Robustez e refatoração (17/11, noite)**: o container ROS caía com segfault em **toda parada**
   da simulação e às vezes **na partida** (callbacks do gz-transport do gimbal, do vento e do
   `GzWorld` publicavam/travavam em membros já destruídos ou ainda não criados; backtrace por core
   dump: `rcl_publish` ← `GimbalControllerNode::onJointState`). Corrigido com `sim::GzCallbackGate`
   (fecha as assinaturas no destrutor; abre no fim do construtor). Refatorações sem mudança de
   comportamento: parâmetros do `pursuit_controller_node` em `pursuit_controller_parameters.cpp`,
   manifesto de replay em `session_recorder_snapshot.cpp`; GCS com roteamento por domínio
   (`param_memory.py`, `sim_ops.py`, `sim_isaac.py`) e painéis `studies_panel.js`/`replay_panel.js`;
   reaplicação dos parâmetros lembrados não roda mais em duplicata. Testes: 213 C++ e 17 da GCS ok.
8. ~~**Limpar os parâmetros lembrados da GCS do Isaac**~~ feito em 17/11 pelo `param.forget`
   (websocket): sobraram só `controller_type=ibvs_mpc` e `mpc.lag.enabled=true`.

## Estado do repositório

- `feat/vertical-modes-gcs` contém **todas** as branches de trabalho (`feat/gcs`, `feat/scenarios`,
  `fix/mpc-oscillation`, `fix/sim-shutdown-remote-viz` são ancestrais lineares). Últimos commits:
  - `c6fa42a` APPROACH: velocidade pela expansão da ROI (tempo até o contato) + distância pela ROI
  - `cf2c922` GCS: drones removidos somem do 3D; omnisciente pelas TFs do mundo
  - `5117940` Malha interna: feedforwards do MPC desligados; aborto só por inclinação
  - `2dbe005` Correção da aproximação cega + saídas thrust_torque/motors do MPC dinâmico
- `centered_mpc`: **obsoleta, não mesclar** (edita `mpc_3d_tracker_node.cpp`, que não existe mais;
  distância por tópico, enable e yaw P já existem na arquitetura atual).
- **Não commitado**: `guia_controle/` inteiro está fora do git (untracked). Nesta sessão foram
  editados `capitulos/10_mpc_dinamico.tex` (saídas, malha interna SO(3), alocação, lição dos
  feedforwards, problema em aberto), o novo `capitulos/12b_centralizacao_fallback.tex` (incluído no
  `main.tex`) e `referencias.bib` (lee2010geometric, lee2009ttc). **O PDF ainda não foi compilado.**

- **Histórico reescrito em 2026-11-05**: `ros2_ws/.vscode/browse.vc.db` (1,2 GB) estava em
  `1b72845` e o GitHub recusava o push. Os 28 commits após `origin/main` foram reescritos sem ele
  (agora no `.gitignore`); `feat/vertical-modes-gcs` está no remoto. O estado antigo ficou em
  `backup/vertical-modes-gcs-pre-filter`; `feat/scenarios` e `fix/mpc-oscillation` ainda apontam
  para os commits antigos (com o arquivo): não dar push nelas; apagar no item 8.

## Tarefas pendentes (em ordem)

1. ~~**Medir de novo o tremor do omnisciente**~~ **feito**. O erro (7 cm, máx. 14 cm) vinha do KF,
   que suavizava a pose exata. Decisão do usuário: o omnisciente não filtra nada. Agora ele publica
   a posição relativa exata das TFs do mundo e a velocidade pela diferença de poses consecutivas
   (covariância zero); o KF só roda na visão. Medido: erro 0,000 m, passo 1,000.
   Script: `ros2_ws/src/aero_pursuit/scripts/omni_tremor.py [dur] [--no-lift]`.
2. ~~**Verificar se a profundidade é usada de verdade**~~ **feito** (d = 5 m, CPU,
   `scripts/vision_depth_check.py [dur] [--zigzag]` após `scripts/test_setup.sh 5`):
   100 % das detecções CONFIRMED com profundidade (parado e zigue-zague ±1 m/s).
   `TargetDetection.depth` fica 0,30–0,44 m abaixo da distância centro a centro (face frontal +
   posição da câmera) e `|estimativa|` 0,15 m abaixo (σ 0,003 m parado, 0,10 m em zigue-zague).
   **Esse viés da face frontal é esperado e deve ficar** (decisão do usuário: não compensar).
   Atenção: a GCS aberta religou o omnisciente e mudou `target_distance` durante o teste;
   feche-a nas medições.
2b. ~~**Oscilação do MPC cinemático (mpc_ltv)**~~ **feito**. O código do MPC e de
   `publishSetpoint` não muda desde `b74480a`; os commits do MPC com empuxo/motores só tocaram o
   caminho do dinâmico. A oscilação vinha do `vertical_mode: center_image` (que a GCS reaplica):
   o feedforward `v_alvo,z = v_medida + v_rel` realimentava a velocidade medida do próprio
   perseguidor, e com o KF omnisciente rápido o drone vibrava (alvo parado: |a| rms 1,6–2,1 m/s²
   contra 0,03 em `los`). Agora há `vertical.feedforward_filter: 0.3` s: |a| rms 0,05–0,06.
   Em zigue-zague o `center_image` ainda varia a altitude (σ 0,4 m) por compensar a arfagem; é o
   comportamento esperado. Medição: `scripts/pursuit_quality.py [parado_s] [zigue_s] [v] [T]`.
3. ~~**Testar em voo o fallback pela ROI**~~ **feito** (`scripts/approach_test.py`; profundidade
   desligada com `vision_tracker sync_slop_sec = -1`). Com `target_size: 0` (antigo) e o alvo parado,
   o perseguidor chegou a 0,53 m: só o tempo até o contato não para o drone (a velocidade de
   equilíbrio é positiva em qualquer distância). Tamanho aparente do X4 medido pela ROI,
   D = sqrt(w h)·r/f: 0,32–0,55 m (mediana 0,43; depende da caixa semeada pela aquisição).
   Padrão agora `fallback.target_size: 0.45`: parado, para a 5,25 m (d = 5); fugindo a 1 m/s,
   acompanha a ~6,6 m. `ttc_*` e `boost_*` mantidos.
   **Depois (pedido do usuário):** o tamanho da ROI é filtrado (`fallback.size_filter`) e o tamanho do
   alvo é aprendido enquanto há profundidade (`fallback.learn_size`, D = 2 r tan(θ/2), média de
   `learn_time` s); `target_size` só vale até aprender. Teste com target_size = 1,0 (errado): aprendeu
   0,60 m (medição independente 0,59) e parou a 5,9 m.
4. ~~**Clima: vento e neblina**~~ **feito**.
   - Vento: `sim/wind_field` (independente de ROS; constante, frente com atraso, rajadas 1−cos,
     turbulência OU, zonas box/cyl inside/outside com borda suave, liga/desliga gradual; 9 testes) +
     sistema do gz-sim `libaero_pursuit_wind_system.so` (F = c_d·v_vento por drone a cada passo,
     carregado em execução pelo `wind_node` via `/world/<w>/entity/system/add`) + `wind_node`
     (`wind.*`, estado em `/weather/wind`). Em voo: 8 m/s do oeste => x500 inclina 5,3° e os dois
     drones derivam para +x; desligar volta ao normal (`scripts/wind_test.py`).
   - Neblina: o ogre2 não tem fog; `fog_filter` (Koschmieder com a profundidade) entre a ponte e os
     consumidores (`/camera/clear` -> `/camera`, `/depth_camera/clear` -> `/depth_camera`), corta a
     profundidade além da visibilidade (`scripts/fog_check.py`).
   - GCS: aba Clima (hoje "Física e clima", com o modelo aerodinâmico). Cenários: bloco `weather:` (`scenarios/weather_front.yaml`). Manual e README.
   - Falta só o texto no guia de controle (item 7).
5. ~~**OpenCL na GPU**~~ **resolvido/diagnosticado**. GPU: AMD RX 6600, OpenCL do Mesa (rusticl),
   limite de 1024 por work-group no dispositivo. O `CL_INVALID_WORK_GROUP_SIZE` vem do `cv::gemm` do
   núcleo (`matmul.dispatch.cpp`: bloco 32×32 escolhido só pelo limite do dispositivo, sem consultar o
   do kernel); o OpenCV cai para a CPU e o resultado continua correto. A rede ViT dá a mesma saída na
   GPU (FP32: 1e-6; FP16: 1e-2). A causa real do "nunca confirma / sem profundidade" era o
   `vision_tracker`: a profundidade ficava no mesmo grupo de callback do RGB e, com a inferência
   lenta, nenhum quadro casava no carimbo. Agora tem grupo próprio + mutex: OpenCL confirma com 100 %
   de profundidade. Mas é mais lento (~14 Hz contra ~24 Hz na CPU), então o padrão virou `cpu`
   (launch, YAML, GCS). Para acelerar de verdade: ROCm OpenCL (precisa de sudo; RX 6600 = gfx1032,
   `HSA_OVERRIDE_GFX_VERSION=10.3.0`) ou OpenCV com Vulkan (`DNN_BACKEND_VKCOM`); CUDA não se aplica (AMD).
   Obs.: `~/.config/aero_gcs/settings.json` ainda guarda `dnn_target: opencl`.
6. ~~**MPC dinâmico ruim**~~ **corrigido**. Causa: o modelo usava o torque como entrada e comandava a
   inclinação prevista 1 passo à frente (≈ atitude atual); a PX4 segue a atitude com ~0,2 s de atraso
   (medido: atitude real 300 ms de parede ≈ 210 ms de simulação atrás do comando), então o comando
   era fraco e atrasado => oscilação de ±2,4 m a 0,2 Hz e σ_z 1,6 m. Novo `mpc.dynamic.attitude_model`:
   `second_order` (padrão; entrada = inclinação, ω̇ = ωn²(θ_cmd−θ) − 2ζωn ω com a taxa medida, ωn = 11,
   ζ = 0,9), `tilt_command` (1ª ordem, τ) e `torque` (antigo). Com peso de entrada < 10 havia ciclo-limite
   de ±8° a ~1 Hz; padrão `weight_torque: 30`. Resultado (d = 5 m, saída attitude): parado σ 6 mm
   (era 1,0 m), zigue-zague σ 0,15 m (era 0,64; cinemático 0,04). Também corrigido: o `offboard_control`
   descartava pares empuxo/torque fora de ordem (agora pareia por `timestamp_sample`); `thrust_torque` e
   `motors` voam (zigue-zague σ ~0,1–0,26 m), com altitude oscilando até ±1 m (empuxo em malha
   aberta pelo modelo de throttle). Atenção: numa sequência rápida attitude → thrust_torque → motors
   o perseguidor caiu uma vez (não reproduzido isoladamente).
7. ~~**Guia de controle**~~ **feito**. Novo capítulo 12c (latência, atrasos e sintonia na prática:
   cadeia medida, casos de atraso no controle, vento como perturbação, tabela de sintonia por método);
   cap. 10 (modelo de atitude de 2ª ordem, pareamento empuxo/torque) e 12b (tamanho aprendido,
   profundidade perdida com inferência lenta, omnisciente sem filtro) atualizados. PDF compilado
   (55 páginas, sem referências indefinidas) num contêiner TeX Live (ver `guia_controle/README.md`).
   `guia_controle/` está no git, com `.gitignore` dos auxiliares.
8. ~~**Merge e push**~~ **feito** (2026-11-05): `main` = `feat/vertical-modes-gcs` (fast-forward) e as duas
   no remoto. Branches locais já contidas foram apagadas. Ficaram:
   - `backup/vertical-modes-gcs-pre-filter`: histórico antes da reescrita (contém o browse.vc.db de
     1,2 GB). **Nunca dar push**; apagar quando não for mais preciso (`git branch -D ...`).
   - `centered_mpc`: obsoleta, também no remoto (`origin/centered_mpc`).
   - Remoto: `origin/feat/gcs`, `origin/feature/dynamic-mpc-joystick`, `origin/fix/sim-cleanup`,
     `origin/test_tcc`, `origin/centered_mpc` continuam lá (todas contidas em `main`, exceto
     centered_mpc). Apagar no GitHub só se o usuário quiser.

## Sessão 2026-11-15: IBVS-MPC (`controller_type: ibvs_mpc`)

Branch `study/tracker-robustness`. MPC não linear com o alvo na imagem dentro do custo
(`control/ibvs_mpc_controller.{hpp,cpp}`): estado p, v, ψ (+ taxa de yaw com 1ª ordem e atraso
compensado), entradas a e ψ̇, Levenberg–Marquardt com Jacobiano encadeado. Publica yaw + yawspeed
(feedforward) e ignora `yaw_mode`/`vertical_mode` (valem só em APPROACH). Métricas novas:
`PursuitMetrics.image_azimuth/image_elevation/rmse_image` (também em session_info e no resumo dos
estudos). GCS: grupo `ibvs.*` na aba Controle. Guia: `guia_controle/capitulos/10b_ibvs_mpc.tex`.

- Malha de yaw da PX4 identificada no SITL: 1ª ordem τ = 0,30 s + atraso 0,16 s. Sem esse modelo, o
  IBVS-MPC fazia ciclo-limite de ±25° no azimute (`ibvs.yaw_time_constant`, `ibvs.yaw_delay`).
- SITL (1 repetição): `scenarios/studies/ibvs_mpc_quick.yaml` (omnisciente) e
  `ibvs_mpc_vision_quick.yaml` (visão). O IBVS reduz rmse_image em 30–60 % e rmse_position em 10–70 %
  em relação a LTV + face_target + center_image, com 5–45 % mais esforço.
- 2026-11-16: estudos com 3 repetições feitos (`ibvs_mpc_full`, `_vision_full`, `_gimbal`); tabelas no
  guia (cap. 10b) com o erro de imagem verdadeiro (`scripts/image_error.py`, TFs do Gazebo no bag).
- Novos, todos com chave `ibvs.enable.*` na GCS: lag de inclinação (neutro no SITL), posição da câmera
  (remove o viés de elevação), viés de inclinação (piorou o figure-8: **desligado por padrão**).
  Gimbal no loop: `ibvs.gimbal_in_loop` + `gimbal_controller mode: mpc` (0,026/0,016 rad no SITL).
- Pendente: rodar `ibvs_mpc_gimbal_loop.yaml` (linha de base MPC + gimbal `track`) e
  `ibvs_mpc_gimbal_loop_vision.yaml` (precisam de `AERO_CAMERA_MOUNT=gimbal`); terminar o zigzag de
  `ibvs_mpc_bias_ab.yaml`; reidentificar yaw_delay/yaw_time_constant em hardware real.
- Um stash "Teleport auto-stash" de 2026-11-16 (stash@{0}) já foi aplicado e commitado; pode ser apagado.

## Sessão 2026-11-07 (noite): gimbal da câmera, layout da GCS e renome

- **Renome**: pasta `~/px4_docker` → `~/aero_pursuit` (symlink `~/px4_docker` mantido por
  compatibilidade) e repositório GitHub `TCC` → `aero-pursuit` (a fazer pelo site; depois trocar o remote para
  `git@github.com:LuccasMarchetti/aero-pursuit.git`). `main` e `feat/gimbal-camera` publicados;
  `tcc/documento-modelo` ficou em `fe0743b` (atrás do main). Docker: o entrypoint limpa `build/`/`install/`
  dos volumes compilados no caminho antigo.
- **Gimbal** (`feat/gimbal-camera`, mesclado em `study/tracker-robustness`): modelo
  `x500_gimbal_depth` (OakD-Lite RGB+profundidade em gimbal de 3 eixos) com `AERO_CAMERA_MOUNT=gimbal`,
  nó `gimbal_controller` (track/stabilize/locked, inércia ligada por padrão), TF dinâmica da câmera,
  estimador consultando a TF no instante da medida. SITL: erro do eixo óptico < 1° (omnisciente) e
  < 1,1° (visão) com alvo a 5 m e 1,5 m/s.
- **GCS**: janelas encostadas dividem a borda ao redimensionar; ocultar/fechar entrega a área às
  vizinhas.
- Manual ilustrado atualizado (`20_perseguicao_d` = gimbal; seletor de câmera e linha Gimbal nas
  capturas). Pendências: estudo comparando fixa × gimbal (mesmo cenário, `mode: locked` vs
  `track`, com `yaw_mode: hold`); `vertical.tilt_compensation` poderia desligar sozinho com gimbal em
  track/stabilize; cenários ainda não escolhem a câmera (é na partida, `AERO_CAMERA_MOUNT`).

## Sessão 2026-11-06: revisão de bugs (3D, centralização, idioma) e manual ilustrado

Feito (branch `tcc/documento-modelo`, sem merge em `main`):
- `center_image` só mantinha a distância: o ajuste lembrado `vertical.camera_pitch_deg = -31` (câmera
  nivelada) mirava o alvo 31° abaixo do centro. Agora a inclinação vem da TF estática
  (`vertical.camera_pitch_source: tf`, padrão). SITL `s3_figure8` omnisciente/asmc: el RMS 33° → 5,6°.
  O ajuste −31 continua em `~/.config/aero_gcs/settings.json` (ignorado com `tf`); o usuário decide se esquece.
- Vista 3D piscando: linhas dinâmicas realocavam buffers a cada quadro (~34/s), z-fighting dos eixos
  das TFs coincidentes e da grade. Corrigido em `scene3d.js`.
- Textos PT no modo EN (rótulos em canvas, estados compostos, descrições dos parâmetros).
- `docs/MANUAL_GCS.md`: tour ilustrado com 30 capturas anotadas (`docs/manual_gcs/`, script em `tools/`).

Pendente:
- **Visão: perseguidor "foge" depois de passar pelo alvo.** No `s3_figure8` com visão (alvo a ~19 m,
  APPROACH sem profundidade) o perseguidor cruzou o alvo e seguiu em APPROACH com o alvo **atrás**
  (az ≈ 180° pela verdade) por 30 s, até 52 m: o TrackerVit continuou com score > 0,35 em outra coisa.
  Investigar um teste de consistência (ex.: ROI sem profundidade com o alvo já perto, salto do rumo)
  antes de confiar no APPROACH longo.
- Descrições dos cenários (`description:` nos YAML) seguem em PT no modo EN.

## Sessão 2026-11-05 (noite): opção A do TCC

Feito e em `main` (branch `tcc/documento-modelo`):
- MPC-PX4 (`mpc.mode: kinematic_lag`, `LagMpcController`), `offboard_control.acceleration_feedforward`,
  laço de sintonia em lote `ros2_ws/tune_study.py` (Nelder–Mead em log θ sobre o `run_study.py`),
  identificação da malha do PX4 `ros2_ws/identify_px4_lag.py`, cenários S1–S4 e especificações em
  `scenarios/tuning/`, estudos `scenarios/studies/opcao_a_*.yaml`.
- GCS em inglês por padrão com seletor PT-BR (`web/js/i18n.js`).
- Comentários e docstrings do código traduzidos para inglês (textos da interface e docs continuam em PT).
- PDF da opção A revisado (cronograma, apêndice B, citação) e com os primeiros resultados SITL.

Pendente:
- `mpc.inner_tau` padrão passou a 0,2 s (identificado no SITL). Comparação S4 omnisciente, 1 ensaio:
  τ = 45 ms → RMSE 1,43 m; τ = 200 ms → 0,50 m; `mpc.adaptive_tau` partindo de 45 ms → 0,58 m.
  Confirmar com degraus (E0) e mais repetições.
- O estimador de τ (`mpc.adaptive_tau`) só aparece no log INFO (silenciado no nível warn): publicar nas métricas.
- Rodar as sintonias completas (8 especificações em `scenarios/tuning/`, ~15 avaliações × 4 ensaios)
  e o E2 completo (10 repetições). Só uma sintonia curta (8 avaliações) e o E2 reduzido rodaram.
- Mensagens do servidor da GCS (Python) e toasts vindos do backend ainda saem em português.

## Próximos passos sugeridos

- Reconstruir a imagem Docker (`./docker/aero.sh build`) para incluir `mesa-opencl-icd`/`clinfo` e o
  novo código (o Dockerfile foi atualizado, a imagem não foi reconstruída nesta sessão).
- MPC dinâmico: zigue-zague ainda pior que o cinemático (σ 0,15 × 0,04 m); compensar o atraso de
  ~0,2 s (predizer o estado inicial) permitiria baixar o peso de entrada. Altitude nas saídas
  thrust_torque/motors oscila até ±1 m (empuxo em malha aberta): integrar o erro vertical.
- Queda do perseguidor numa troca rápida attitude → thrust_torque → motors em voo (não reproduzida).
- `~/.config/aero_gcs/settings.json` guarda `dnn_target: opencl` e overrides antigos
  (`target_distance 2.1`, `max_vel 4.4`, `max_acc 5.3`, `center_image`).

## Armadilhas conhecidas

- **Configurações lembradas da GCS**: `~/.config/aero_gcs/settings.json → param_overrides`
  reaplica, sempre que o nó aparece, `mpc.mode=kinematic`, `target_distance=2.1`, `max_vel=4.4`,
  `max_acc=5.3`, `vertical_mode=center_image`. Nos testes, defina os parâmetros explicitamente
  depois da subida.
- **Inferência**: `cpu` é o padrão e o mais rápido nesta máquina (item 5).
- **Subir a simulação** (headless, logs INFO):
  ```bash
  cd ros2_ws && ./sim_manager.sh stop; ./stop_simulation.sh
  WORLD_NAME=lawn SKIP_PX4_BUILD=1 GZ_HEADLESS=1 GZ_RENDER_API=opengl LOG_LEVEL=info FOXGLOVE=0 \
    ./sim_manager.sh start controller:=mpc mpc_mode:=kinematic yaw_mode:=face_target \
    dnn_target:=cpu tf_source:=gazebo record:=false training:=false joystick:=false gcs_stream:=true
  # espere "Decolagem concluída" em logs/<data>/aero_pursuit.log
  ```
- **Levantar o X4**: publique `std_msgs/Bool true` em `/X4/enable` e
  `geometry_msgs/Twist` com `linear.z=1.5` por 3 s em `/X4/gazebo/command/twist`.
- **Omnisciente**: publique `aero_pursuit/msg/TargetTfCmd {target_frame: X4,
  enable_tf_tracking: true}` em `/target_tf_cmd`.
- **`ros2 param set` pela CLI leva 1–3 s**; scripts de teste não devem contar com troca instantânea.
- **A GCS roda à mão no terminal do usuário** (`./ros2_ws/gcs.sh`). Mudanças em Python exigem
  reiniciá-la; o JS é servido do código-fonte.
- **O modo automático do Claude Code bloqueou os comandos de shell** no fim da sessão anterior;
  use o modo manual/padrão.
- **CLAUDE.md**: não ler os arquivos internos de `px4_msgs` (use `ros2 interface show`). Rodar
  `graphify update .` depois de mudanças no código. Commits com o trailer de coautoria.
