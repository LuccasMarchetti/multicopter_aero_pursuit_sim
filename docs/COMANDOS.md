# Comandos: GCS em cada computador e o lançador

Cada computador que roda simulação tem a **sua** GCS, instalada pelo mesmo script e com os mesmos
comandos. As GCSs não sabem umas das outras. Na aba Simulação, cada uma escolhe entre Gazebo e
Isaac Sim (o Isaac precisa de GPU NVIDIA RTX). O computador que só **acessa** roda o
**lançador**: uma página que procura na rede as GCSs que ele alcança e abre a que você clicar.

```
 computador A                computador B (GPU NVIDIA)     computador de acesso     
 ┌───────────────────┐       ┌────────────────────┐        ┌──────────────────────────┐
 │ GCS  Gazebo|Isaac │       │ GCS  Gazebo|Isaac  │  <───  │ aero-launcher            │
 │ aero ...          │       │ aero ...           │  <───  │ http://localhost:8090/   │
 └───────────────────┘       └────────────────────┘        └──────────────────────────┘
        independentes, sem ligação entre si                 varre a rede e redireciona
```

## 1. Instalar a GCS num computador

```bash
git clone --recursive git@github.com:LuccasMarchetti/TCC.git ~/aero_pursuit
cd ~/aero_pursuit
./aero install            # detecta o modo; --native ou --docker forçam um
```

| Modo | Quando | O que instala |
|---|---|---|
| **nativo** | ROS 2 Jazzy no host e o workspace compilado (`./install.sh`) | serviço systemd do usuário `aero-gcs` (roda `ros2_ws/gcs.sh`) |
| **docker** | sem ROS no host (ex.: computador compartilhado, sem root) | imagem `aero-pursuit` (Gazebo + GCS, ~30–50 min na 1ª vez) e o contêiner `aero-gcs` |

Nos dois modos, `aero install` também instala o comando `~/.local/bin/aero`, o atalho "Aero GCS"
no menu e o início automático. Depois sobe a GCS. Configuração gravada em `~/.config/aero/aero.env`
(modo e, se definido na instalação, `ROS_DOMAIN_ID`: `ROS_DOMAIN_ID=N ./aero install` isola a
simulação Gazebo num computador compartilhado; o Isaac usa sempre o domínio 42).

**Isaac Sim.** Com GPU RTX, `aero install` avisa o que falta: `aero isaac setup` (uma vez: Isaac
5.1, Pegasus e PX4 no host; pede sudo para ninja/ccache) e `aero build` (o lado ROS do Isaac roda do mesmo workspace
que o do Gazebo). Daí em diante o seletor da aba Simulação aceita o Isaac. No modo docker, o
contêiner da GCS fala com o Isaac do host pelo agente `isaac/host_agent.py` (socket Unix, só os
subcomandos de `isaac/host.sh`), que `aero gcs start` sobe junto. O Isaac em si roda nativo no
host.

**GPU NVIDIA no Docker sem o NVIDIA Container Toolkit** (computador sem root): `docker/aero.sh`
e `isaac/host.sh` usam `docker/nvidia_host.sh`, que passa ao contêiner os dispositivos
`/dev/nvidia*` e as bibliotecas da versão do driver em uso no host, e limita o EGL ao fornecedor
NVIDIA. Com isso o Gazebo renderiza na GPU (o Mesa não consegue usar placas NVIDIA) e o vídeo da
GCS sai pelo NVENC. O arquivo é regerado a cada `aero gcs start`, então uma atualização do driver
vale sem reconstruir a imagem. Com o toolkit instalado, usa-se o `compose.nvidia.yaml` normal.

**Atualizar o código:** `aero update` (git pull + submódulos + reinicia a GCS). Se mudou C++/ROS:
`aero build` (vale para o Gazebo e para o Isaac).

## 2. Comandos (iguais em todo computador)

| Comando | O que faz |
|---|---|
| `aero status` | modo, GCS no ar, início automático, simuladores disponíveis, simulação e RTF |
| `aero view` | abre esta GCS no navegador (sobe se estiver parada) |
| `aero gcs start` / `stop` / `restart` | sobe / para / reinicia a GCS (no docker, também o agente do Isaac) |
| `aero gcs status` / `logs` / `token` | estado / log ao vivo / token de acesso pela rede |
| `aero autostart on\|off\|status` | GCS no boot (ver §4) |
| `aero sim gazebo` / `isaac` | troca o simulador desta GCS (ela reinicia; só com a simulação parada) |
| `aero sim start` / `stop` / `status` | simulação com o formulário salvo na aba Simulação |
| `aero study a.yaml [b.yaml …] [--repetitions N]` | estudos em cascata (`ros2_ws/run_study.py`) |
| `aero access` | links desta GCS (local, rede, túnel Cloudflare) e o token |
| `aero logs gcs\|sim\|world\|ros\|px4\|agent\|study` | log ao vivo: GCS; Gazebo; mundo Isaac; lado ROS do Isaac; PX4 do Isaac; agente; estudo |
| `aero build [pacotes]` | `colcon build` (nativo: `ros2_ws/`; docker: no volume do contêiner) |
| `aero image` | (docker) recompila a imagem `aero-pursuit` |
| `aero update` | git pull, submódulos e reinício da GCS |
| `aero isaac setup` | instala o Isaac Sim, o Pegasus e o PX4 no host (`aero isaac build` = `aero build`) |
| `aero isaac status` / `up` / `down` / `px4 <cmd>` | Isaac deste computador direto (sem a GCS; `isaac/host.sh`) |
| `aero launcher` | abre o lançador (§3) também neste computador |
| `aero uninstall` | desliga o início automático, para a GCS e tira o atalho |
| `aero pursuer <args>` | perseguidor na Raspberry Pi ([RASPBERRY.md](RASPBERRY.md)) |

Simulações e estudos continuam rodando com o navegador fechado. No modo nativo, reiniciar a GCS
não derruba a simulação (`KillMode=process`). No modo docker, o Gazebo roda dentro do contêiner da
GCS e cai junto com ele; o Isaac roda no host e continua.

## 3. Lançador (computador de acesso)

Só precisa de Python 3.9+. Não precisa de ROS nem do repositório: basta o arquivo
`launcher/aero_launcher.py`.

```bash
./launcher/install.sh            # aero-launcher + "Aero GCS" no menu (--autostart: abre no login)
aero-launcher                    # http://localhost:8090/ no navegador
python3 aero_launcher.py         # ou direto, sem instalar (qualquer sistema)
```

O que ele procura, ao abrir e a cada minuto (botão "Search again" força):
- as redes /24 das interfaces deste computador (rede local, ZeroTier);
- os peers online do Tailscale (`tailscale status --json`);
- este computador e os endereços salvos na página (`~/.config/aero_launcher/hosts.txt`), para
  GCSs fora dessas redes (`host` ou `host:porta`).

Um endereço conta como GCS quando `https://<endereço>:8443/api/info` responde (público, sem
token). A mesma GCS achada por vários caminhos vira um cartão só, com as rotas em ordem de
latência (cada rota mostra a interface por onde foi achada). **Clicar no cartão** abre a GCS pela rota mais rápida, e os links menores abrem por uma
rota específica. A GCS do próprio computador abre por `http://localhost`, sem token.

No primeiro acesso a cada GCS, o navegador avisa do certificado autoassinado e a GCS pede o token
(`aero gcs token` no computador dela). Depois fica um cookie de 30 dias naquele navegador. O
lançador não guarda token nenhum.

## 4. Início automático

`aero autostart on` (já feito pelo `aero install`) escolhe sozinho:
- **nativo com linger** (`loginctl enable-linger`): serviço `aero-gcs` habilitado, que
  sobe no boot sem login;
- **senão** (modo docker, ou sem permissão para o linger): linha `@reboot` no crontab
  do usuário (`crontab -l`), que espera 45 s pelo Docker e roda `aero gcs start`. Log em
  `~/.local/state/aero/gcs_autostart.log`.

## 5. Onde fica cada coisa

| | |
|---|---|
| `~/.config/aero/aero.env` | modo da instalação e `ROS_DOMAIN_ID` |
| `~/.config/aero_gcs/` | configuração, token e certificado da GCS (montado no contêiner no modo docker) |
| `~/.local/state/aero/isaac/` | logs do Isaac (`logs/world.log`, `ros.log`, `host_agent.log`), socket do agente e sockets de vídeo (`run/`) |
| `datasets/`, `scenarios/` | no próprio repositório, nos dois modos |

## 6. Problemas comuns

- **O lançador não acha uma GCS.** No computador dela: `aero status`. Daqui: o endereço responde
  (`curl -k https://<ip>:8443/api/info`)? Se a GCS estiver fora das redes varridas, adicione o
  endereço na página.
- **A GCS não sobe depois do boot.** `aero status`, `aero gcs logs`. Com cron:
  `cat ~/.local/state/aero/gcs_autostart.log`.
- **1º `aero gcs start` no modo docker demora.** Ele compila o workspace no volume do contêiner
  (alguns minutos). Acompanhe com `aero gcs logs`.
- **Isaac aparece "indisponível" no seletor.** O motivo aparece ao lado: sem GPU RTX, Isaac não
  instalado (`aero isaac setup`) ou, no modo docker, agente parado (`aero gcs restart`).
- **Mudança em nó C++ não aparece no Isaac.** Recompile com `aero build` e reinicie a simulação.
- **Recurso novo "sumiu" da página.** É JavaScript em cache: recarregue com Ctrl+Shift+R.
