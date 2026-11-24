# 6. Plano de implementação proposto

Este é só o plano. Nada de ROS ou Gazebo foi alterado neste estudo. Pelas regras do projeto: branch
própria (`feat/aero-body-model`), dependências novas também em `install.sh` e `docker/Dockerfile`,
parâmetros visíveis na GCS e escolha pelo usuário (modelo ligado/desligado, nível) em vez de algo
fixo.

## Fase 0: experimento de 1 dia, sem código C++

1. Copiar `x500_depth` para `aero_pursuit/gz_models/x500_depth_aero/` com o bloco `Hydrodynamics` de
   [05 §5.3](05_viabilidade_e_desempenho.md) (CdA_x = 0,03).
2. Voar os cenários `reference` e `figure8_basic`, mais um degrau de velocidade a 10 m/s, com e sem
   o bloco.
3. Medir: velocidade de regime × tilt, overshoot e tempo de frenagem do perseguidor, erro de
   rastreamento, tilt pedido (`/pursuit/metrics`), RTF.
4. **Critério para seguir:** se as métricas da perseguição mudarem de forma relevante (erro, tilt,
   saturação), o modelo de corpo vale a fase 1. Se não mudarem, documentar que, na faixa de 2–5 m/s,
   o modelo do Gazebo basta, o que também é um resultado válido para o TCC.

## Fase 1: plugin `AeroBodySystem` (corpo tabelado + vento coerente)

Estrutura no pacote (segue o estilo `.hpp`/`.cpp` do projeto, RAII, sem ROS dentro do plugin):

```
ros2_ws/src/aero_pursuit/
  include/aero_pursuit/gz_aero/
    aero_table.hpp          # grade uniforme α×β(×V), interpolação, periodicidade, carga YAML
    body_aero_model.hpp     # v_ar → (F, M): tabela + amortecimento + corte em V baixa
    wind_field.hpp          # constante + Dryden (semente fixa) + interface para esteira
  src/gz_aero/
    aero_table.cpp  body_aero_model.cpp  wind_field.cpp
    aero_body_system.cpp    # gz::sim::System (Configure/PreUpdate), registra via GZ_ADD_PLUGIN
  config/aero/
    x500_body_aero.yaml  x3_body_aero.yaml  x4_body_aero.yaml
  tools/
    gen_aero_table.py       # painéis Newtonianos sobre malha/SDF + escala (fase 1b)
  test/
    test_aero_table.cpp     # interpolação, periodicidade, simetrias, polos β = ±90°
    test_body_aero_model.cpp# placa plana analítica, V → 0, vento puro, sinais FLU
```

- `aero_table`, `body_aero_model` e `wind_field` **não dependem do Gazebo**. Ficam testáveis em gtest
  e reutilizáveis pelo MPC (o mesmo modelo prediz e simula).
- `aero_body_system` é a casca fina sobre a ECM. CMake: `find_package(gz-sim8)` só para esse alvo.
  Instalar em `lib/` e exportar `GZ_SIM_SYSTEM_PLUGIN_PATH` em `launch_gz_and_drones.sh`.
- Ativação como *world system*: um `server.config` do projeto que inclui o do PX4 mais o nosso
  plugin, com a lista de veículos. O mundo `lawn` continua sendo o do PX4.
- **Wind:** o plugin escreve o vento atual (média + Dryden) na entidade `Wind`. O
  `MulticopterMotorModel` passa a ver a mesma rajada sem modificação.
- **Parâmetros em runtime** (gz-transport, ponte ROS para a GCS): `enabled`, `scale_cda`, `wind_mean`,
  `turbulence_intensity`, `seed`. Telemetria limitada a 20 Hz: α, β, V_ar, F, M, vento.
- **TF/Foxglove:** nenhuma TF nova é necessária. Se quiser, um `MarkerArray` de seta para F_aero e
  vento no frame `x500_depth_0`.

Entregáveis da fase 1:

1. Tabela do x500 gerada pelo `gen_aero_table.py` (fonte B) e escalada para CdA_x(α=0) = 0,030 m²,
   com a curva Drag/q(α) comparada com a Fig. 30 da NASA (Iris+).
2. Gráficos de velocidade de regime × tilt (modelo do Gazebo × fase 1 × NASA/ArduPilot) e da resposta
   a rajada.
3. MPC dinâmico e planeza com D_eff configurável (ver [05 §5.4](05_viabilidade_e_desempenho.md)),
   sem mudar o padrão sem consultar o usuário.

## Fase 2 (opcional): correções de rotor

- ΔT(μ, λ_c), H-force e flapping por tabela BEMT da hélice 1045 do x500 (ou Gill & D'Andrea 2019),
  somados no link do rotor lendo a velocidade da junta. O `MulticopterMotorModel` continua no lugar
  e, se o H tabelado substituir o rotor drag linear, `rotorDragCoefficient` vai para 0 numa cópia
  do modelo.
- Efeito solo (Cheeseman–Bennett) com altura por *ray cast* ou pela altura do terreno plano.
- VRS: queda de empuxo e ruído de empuxo na região de descida crítica (modelo ONERA,
  Talaeizadeh 2019/2020). É relevante para os modos verticais da GCS.

## Fase 3 (opcional): esteira do alvo

Cone de downwash do alvo (v_i ≈ √(T/(2ρA)) decaindo com a distância, forma de Kiran et al. 2025)
somado ao vento local do perseguidor. Permite estudar perseguição por baixo ou atrás e degradação
de empuxo. É um diferencial específico de ar-ar.

## Validação (todas as fases)

| Teste | Referência |
|---|---|
| V → 0 e vento = velocidade do drone | força nula |
| Placa plana normal analítica | ½ρ C_D A V² com C_D ≈ 1,17 |
| Velocidade de regime × tilt | NASA (Iris+), ArduPilot (V_ref, θ_ref) e, se houver, voo real do x500 em log `.ulg` |
| PX4 EKF2 com `EKF2_DRAG_CTRL = 1` | estimativa de vento converge para o vento imposto |
| RTF | sem queda mensurável (`gz stats`) |
| Determinismo | mesma semente gera a mesma rajada (repetibilidade dos experimentos do TCC) |

## Estimativa de esforço

| Fase | Esforço |
|---|---|
| 0 | ~1 dia |
| 1 (plugin + testes + tabela B + integração GCS) | ~1–2 semanas |
| 1b (calibração com voo real do x500, se houver Pixhawk/logs) | ~2–3 dias |
| 2 | ~1–2 semanas |
| 3 | ~1 semana |
| CFD do corpo (validação opcional) | 1–3 semanas, mais tempo de CPU |
