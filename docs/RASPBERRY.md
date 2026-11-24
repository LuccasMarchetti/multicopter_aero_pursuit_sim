# Perseguidor embarcado na Raspberry Pi 5

Os nós do perseguidor (rastreador, estimador, controlador, gimbal e offboard da PX4) rodam num
container na Raspberry Pi 5. O mundo (Gazebo, PX4 SITL + agente XRCE, alvo, cenário, gravação e GCS)
continua no PC e só troca mensagens ROS 2 com ela pela rede. É o mesmo código do PC, compilado sem a
parte de simulação (`AERO_SIM=OFF`).

## Uso

```bash
# PC: simulação sem os nós do perseguidor (eles rodarão na Pi)
AERO_PURSUER_HOST=remote ./ros2_ws/sim_manager.sh start      # + AERO_CAMERA_MOUNT=gimbal se for o caso

# Pi (comandos dados do PC, por SSH com chave; host = usuário@IP da Pi)
export AERO_PI_HOST=usuario@ip-da-pi
docker/pursuer.sh build            # 1ª vez ~40 min (OpenCV); depois ~7 min (só o aero_pursuit)
docker/pursuer.sh buildlog         # acompanha a compilação
docker/pursuer.sh run              # sobe o container (rede do host, DDS na LAN)
docker/pursuer.sh logs | stats | shell | stop

# PC: estudos e GCS funcionam como antes (os parâmetros vão pela rede)
./ros2_ws/run_study.py scenarios/studies/pursuer_split_quick.yaml
```

Pela GCS (Gazebo): aba Simulação → "Execução do perseguidor" = remoto, host `usuário@IP` (vazio =
`AERO_PI_HOST` ou a Pi padrão). **Iniciar** sobe o simulador com `AERO_PURSUER_HOST=remote` e depois
`pursuer.sh run` com o controlador, MPC, saída, yaw, vertical, inferência, mundo e câmera do
formulário; **Parar** para o contêiner remoto antes do simulador. Em Execução aparece o estado do
host a cada 5 s por SSH (contêiner, CPU, memória, temperatura e a latência da rede medida pela
conexão TCP à porta 22), com botões para subir/parar só o perseguidor e o log `pursuer`
(`docker logs -f`). "Parar a simulação se a latência passar de" (ms, 0 = nunca) para tudo após três
leituras seguidas acima do limite. Isaac Sim: ainda não (o `isaac_world.launch.xml` não tem modo
remoto e a Pi não está na mesma rede DDS do computador do Isaac).

Sem host padrão: `AERO_PI_HOST` ou 2º argumento (na GCS, o campo "Host remoto"). O mesmo par
simulador/perseguidor também roda numa máquina só: `AERO_PURSUER_HOST=remote` no simulador e
`ros2 launch aero_pursuit pursuer.launch.xml` em outro terminal.

## Placa e preparação

| Item | Valor |
|---|---|
| Placa | Raspberry Pi 5 Model B Rev 1.1, 4 núcleos Cortex-A76, 7,8 GiB, NVMe 235 GB |
| SO | Ubuntu 24.04.4 LTS aarch64, ROS 2 Jazzy base (não usado: tudo vem na imagem) |
| Rede | Ethernet gigabit, 0,4 ms de ping até o PC; relógios por NTP |
| Docker | `docker.io` 29.1 + `docker-buildx` (apt), usuário no grupo `docker` |
| Kernel | `/etc/sysctl.d/60-ros-large-data.conf`: `net.core.{r,w}mem_{max,default}=4194304` |

O buffer de socket padrão (212 KB) é pequeno para mensagens de imagem pelo Fast DDS; no PC o
`rmem_max` já era 4 MB.

## O que mudou no código

| Mudança | Onde | Por quê |
|---|---|---|
| `AERO_SIM` (CMake, padrão ON) | `CMakeLists.txt` | OFF compila só os nós de bordo, sem Gazebo/GStreamer |
| `joint_interface: gz \| ros` | `gimbal_controller` | `ros`: estado em `/gimbal/joint_states`, comando em `/gimbal/cmd/{yaw,roll,pitch}` (Float64) e `/gimbal/joint_command` (JointState). Serve também para Isaac Sim e hardware |
| `pursuer_host: local \| remote` | `aero_pursuit_sim.launch.xml` (`AERO_PURSUER_HOST`) | remote: não sobe os nós do perseguidor; sobe `image_downscaler` e a ponte das juntas do gimbal. `AERO_REMOTE_IMAGE_SCALE` (padrão 1.0), `AERO_REMOTE_JPEG_QUALITY` (80) |
| `image_downscaler` | novo nó (lado do simulador) | RGB em JPEG (q 80) e profundidade em PNG 16 bits (mm, sem perdas), ambas em resolução cheia por padrão; escala opcional com intrínsecos escalados |
| `image_compressed`, `depth_compressed`, `image_qos_reliable` | `vision_tracker_node` | entrada JPEG e PNG; QoS confiável para imagens que cruzam processos |
| `pursuer.launch.xml` | novo | só os nós de bordo, com os tópicos remotos |
| repetição na aplicação de parâmetros | `scenario_runner` | o serviço de parâmetros do nó remoto pode ainda não ter sido descoberto no início do estudo |
| `Dockerfile.pursuer`, `pursuer.sh` | `docker/` | imagem multi-arquitetura (arm64 na Pi), compilada na própria placa |

## Problemas encontrados no caminho

1. **Imagem crua em best effort perde quadros inteiros.** Com o perseguidor em outro processo, o
   rastreador processava ~9 quadros/s em vez de ~29: um quadro de 1,5 a 6 MB vira dezenas de
   fragmentos UDP, e a perda de um descarta o quadro. O verificador de aparência suspendia o
   rastreamento e a profundidade quase nunca era associada. QoS confiável resolveu no PC
   (29,5 quadros/s).
2. **Imagem crua pela rede atrasa o controle.** Na Pi, ~80 MB/s de RGB + profundidade confiáveis
   ocupavam a thread de recepção do Fast DDS, e os setpoints chegavam ao PX4 com até 200 ms de
   idade; no modo onisciente o drone chegou a bater no chão. Primeiro resolvido com RGB e
   profundidade reduzidas à metade (~12 MB/s); depois sem reduzir: RGB 1920×1080 em JPEG
   (~61 kB/quadro) e profundidade 640×480 em PNG 16 bits (~7 kB/quadro) somam ~1,5 MB/s.
   H.264/H.265 não foi necessário: o JPEG das cenas do Gazebo já é pequeno, e a Pi 5 não tem
   codificador de vídeo em hardware.
3. **Parâmetros do primeiro ensaio não aplicados.** O `scenario_runner` desistia se o nó remoto
   ainda não estivesse descoberto; agora tenta de novo por até 15 s.
4. **Resolução reduzida degradava a visão.** Com 0,5× o rastreio com gimbal caiu para 56% (3 perdas)
   e a distância com câmera fixa piorou (1,89 m). Em resolução cheia os dois voltam ao nível do PC
   (tabela abaixo); a hipótese de defasagem da TF do gimbal não se confirmou.

## Resultados (s3_figure8, 45 s, 1 repetição)

Métricas de verdade de campo (`latex/monografia/scripts/metricas_ensaio.py`), as mesmas dos
estudos do TCC. "Local" = tudo no PC (E3 e estudos de 16–17/11). "Pi" = resolução cheia (17/11,
estudos `pursuer_split_quick_20261117_012719`, `pursuer_pi_ibvs_20261117_013824`,
`pursuer_pi_gimbal_20261117_014226`); "Pi 0,5×" = rodada anterior, imagens reduzidas à metade.

| Ensaio | Onde | Dist. RMS | Imagem RMS | No FOV | Rastreio | Perdas | ms/ciclo (média / p99) |
|---|---|---|---|---|---|---|---|
| LTV + center_image, onisciente | Local (E3) | 0,62 m | 0,184 rad | 100% | — | 0 | 0,32 |
| | **Pi** | 0,62 m | 0,194 rad | 100% | — | 0 | 0,25 / 0,60 |
| LTV + center_image, visão | Local | 1,16 m | 0,243 rad | 100% | 86% | 0 | — |
| | Pi 0,5× | 1,89 m | 0,229 rad | 100% | 83% | 0 | 0,27 / 0,81 |
| | **Pi** | 0,97 m | 0,250 rad | 100% | 86% | 0 | 0,23 / 0,69 |
| IBVS perception-aware, onisciente | Local (E3) | 0,56 m | 0,064 rad | 100% | — | 0 | 5,3 |
| | **Pi** | 0,59 m | 0,066 rad | 100% | — | 0 | 4,1 / 9,0 |
| IBVS clássica, onisciente | Local (E3) | 1,68 m | 0,084 rad | 100% | — | 0 | 1,5 |
| | **Pi** | 1,69 m | 0,084 rad | 100% | — | 0 | 1,1 / 2,4 |
| Gimbal `track` (LTV), onisciente | Local | 0,61 m | 0,295 rad | 90% | — | 0 | 0,43 |
| | **Pi** | 0,60 m | 0,299 rad | 90% | — | 0 | 0,26 / 0,63 |
| IBVS com gimbal no laço, onisciente | Local | 0,41 m | 0,019 rad | 100% | — | 0 | 9,2 |
| | **Pi** | 0,41 m | 0,020 rad | 100% | — | 0 | 6,1 / 11,0 |
| Gimbal `track` (LTV), visão | Local | 0,96 m | 0,072 rad | 100% | 86% | 0 | — |
| | Pi 0,5× | 3,35 m | 0,581 rad | 73% | 56% | 3 | 0,28 / 0,84 |
| | **Pi** | 0,95 m | 0,092 rad | 100% | 86% | 0 | 0,29 / 1,33 |

Em resolução cheia o rastreador na Pi publica ~29 detecções/s (1356–1358 em 47 s), com a câmera
fixa e com o gimbal: o JPEG 1080p é decodificado na Pi sem limitar a taxa, e não foi preciso
travar a simulação pela latência (na rodada 0,5× eram ~23 quadros/s com o gimbal). Rodada 0,5×:
~1,6 núcleo com a câmera fixa e ~2 com o gimbal, 56–62 °C, sem *throttling*
(`vcgencmd get_throttled` = 0x0), ~100 MB de RAM no container. O MPC mais caro (IBVS com gimbal,
11–13 ms no p99) usa um quarto do período de 50 ms. Os
tempos na Pi são menores que no PC porque lá o Gazebo disputa a CPU.

Estudos: `pursuer_split_quick.yaml`, `pursuer_pi_ibvs.yaml`, `pursuer_pi_gimbal.yaml`.

## Pendências

- CPU e temperatura da Pi em resolução cheia não foram medidos de novo (rodada 0,5×: ~1,6–2 núcleos).
- O modo `track` do gimbal tem erro de imagem de ~0,3 rad também no PC; é do próprio modo, não da
  Pi.
- Seleção manual do alvo pela GCS no modo remoto com `AERO_REMOTE_IMAGE_SCALE` < 1: o clique vem
  na resolução cheia e o rastreador usa a reduzida (no padrão 1.0 não há diferença).
- Hailo (13 TOPS): não necessário até aqui; o ViT na CPU sustenta ~29 quadros/s em 1080p.
- Depois: Arduino Uno Q / Raspberry Pi Zero 2W.
