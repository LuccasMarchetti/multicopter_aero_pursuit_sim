# Perseguição no Isaac Sim: plano

> **Estado (17/11/2026):** etapas 1–7 implementadas e testadas (resultados em `docs/ISAAC_SIM.md`).
> Como rodar e o que foi adaptado: `docs/ISAAC_SIM.md`. Pela ZeroTier o RTT é ~240 ms (relay),
> então a malha de controle inteira roda no bismark e este PC só comanda e assiste.

Objetivo: repetir a perseguição de um drone por outro no NVIDIA Isaac Sim, com os mesmos nós do
perseguidor que rodam no Gazebo e na Raspberry Pi (`launch/pursuer.launch.xml`). É mais uma
etapa do TCC: o controle e a percepção não mudam; muda o simulador (renderização, física,
sensores). Fica de fora o que existe só no Gazebo: física aerodinâmica própria, vento, névoa,
enxame, treinamento por reforço, iscas.

Máquina: PC remoto com GPU NVIDIA RTX (exigência do Isaac Sim), acessado pela ZeroTier (bismark, `docs/ISAAC_SIM.md`).
Este PC (ou a Pi) roda o perseguidor e a GCS e vê tudo pela rede, como no modo
`AERO_PURSUER_HOST=remote`.

## O que precisa existir no Isaac Sim

| Item | Gazebo hoje | Isaac Sim | Observação |
|---|---|---|---|
| Perseguidor | `x500_depth` / `x500_gimbal_depth` | Multirrotor do Pegasus (Iris) reparametrizado como x500 | massa, inércia, coeficientes dos rotores de `control/vehicle_params.hpp` |
| PX4 | SITL + gz bridge, uXRCE-DDS | SITL ligado ao Pegasus (MAVLink HIL, a verificar) + uXRCE-DDS igual | os tópicos `/fmu/*` não mudam |
| Câmera RGB | IMX214 1920×1080, 69° | Camera prim + OmniGraph "ROS2 Camera Helper" | mesmos tópicos `/camera/...`; resolução reduzida se a rede pedir (`image_downscaler`) |
| Profundidade | OV7251 640×480 | `distance_to_image_plane` do mesmo helper | 32FC1 em metros, como hoje |
| Gimbal | 3 juntas + JointPositionController | articulação com 3 juntas de revolução + "Articulation Controller" | `gimbal_controller joint_interface:=ros` já publica `sensor_msgs/JointState` em `/gimbal/joint_command` e lê `/gimbal/joint_states` |
| Alvo | X4 do PX4 em trajetória do cenário | segundo multirrotor, sem PX4: pose cinemática a partir da trajetória | `scenario_runner` só para o alvo, ou um script do Isaac lendo o mesmo YAML |
| Verdade de campo | TF do Gazebo (`gz_tf_bridge`) | OmniGraph "ROS2 Publish Transform Tree" | frames com os mesmos nomes (`lawn`, `x500_depth_0`, `X4`) mantêm o modo onisciente |
| Relógio | `/clock` do Gazebo | "ROS2 Publish Clock" | `use_sim_time` como hoje |
| Gravação | `session_recorder` | o mesmo, no PC local | as métricas (`metricas_ensaio.py`) usam a TF de verdade |

## Etapas

1. **Ambiente no PC remoto.** Isaac Sim 5.1 + Pegasus 5.1.0 (a versão do Pegasus acompanha a do
   Isaac; as anteriores não são compatíveis). ROS 2 Jazzy pelo bridge interno do Isaac. PX4
   v1.18 (o mesmo commit do `docker/Dockerfile`). Teste: o exemplo de PX4 do Pegasus decola.
2. **Rede.** Isaac + PX4 + agente XRCE no remoto; perseguidor e GCS neste PC, pela Tailscale. A
   descoberta do DDS por multicast não atravessa a Tailscale: perfil do Fast DDS com
   *initial peers* (os dois IPs) ou Discovery Server. Validar com `ros2 topic hz /clock` e
   `/fmu/out/vehicle_odometry` daqui.
3. **x500 no Isaac.** Ajustar o veículo do Pegasus para os parâmetros do x500 já usados pelo MPC
   dinâmico e pela alocação. Validar como foi feito no Gazebo: identificar a malha de velocidade
   e a de yaw com `identify_px4_lag.py` (e `--yaw`) e comparar com 0,2 s e 0,30 s + 0,16 s.
4. **Câmera fixa.** RGB + profundidade na pose da câmera do x500_depth (0,12; 0,03; 0,24 m), FOV
   de 69°. TF estática igual (`camera_link`, `camera_optical`). Primeiro teste com detecção
   onisciente (não depende da câmera), depois com o rastreador.
5. **Alvo.** Segundo veículo seguindo `s3_figure8` e `s4_zigzag` cinematicamente. Sem PX4 no alvo,
   o que é mais leve e suficiente para a perseguição.
6. **Gimbal.** Três juntas na ordem do `x500_gimbal_depth` (yaw, roll, pitch), comandadas pelo
   `gimbal_controller` com `joint_interface:=ros`. É a interface que já foi separada do Gazebo para
   a Pi.
7. **Estudos.** E3 reduzido (LTV + center_image, IBVS perception-aware e clássico) nos dois
   cenários, onisciente e visão, uma repetição. Comparar com o Gazebo: mesmas métricas, mesmo
   script.

## Riscos e decisões em aberto

- Pegasus + PX4: o modo de ligação (MAVLink HIL com lockstep) e o desempenho com câmera de
  1920×1080 precisam ser medidos; se o tempo real cair muito, reduzir a resolução na origem.
- Não há x500 pronto no Isaac: o modelo será aproximado (geometria visual simples, parâmetros
  físicos do x500). Diferenças de aerodinâmica com o Gazebo devem ser declaradas no texto.
- A troca de simulador pode mudar as constantes das malhas da PX4; por isso a reidentificação da
  etapa 3 vem antes dos estudos.
- Licença/instalação do Isaac Sim no PC remoto e acesso pela Tailscale ainda não existem.

## Referências a conferir antes de escrever no TCC

- Pegasus Simulator: Jacinto et al., arXiv:2307.05263 (2023); documentação e notas de versão em
  https://pegasussimulator.github.io/PegasusSimulator (v5.1.0 para Isaac Sim 5.1.0, 26/10/2025).
- PX4 uXRCE-DDS: https://docs.px4.io/main/en/middleware/uxrce_dds.html (agente 2.4.3 para Jazzy).
