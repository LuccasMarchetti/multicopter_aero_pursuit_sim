# Estudo: aceleração do alvo no filtro e no controle

Pergunta: vale estimar a aceleração do alvo e passá-la adiante (filtro → controle)? Melhora algo?

## O que foi implementado (desligado por padrão)

- **Filtro** (`/target_estimator_node kf_model`): `cv` (velocidade constante, padrão) ou `singer`:
  estado `[p_rel, v_rel, a_alvo]`, aceleração como processo de Gauss-Markov de 1ª ordem
  (σ_a = `singer.accel_std`, τ = `singer.accel_tau`; τ → ∞ ≈ aceleração constante, τ → 0 ≈ CV). A
  variação de velocidade do perseguidor continua entrando como entrada conhecida. Publica
  `/target_3d_accel_filtered`.
- **Omnisciente**: aceleração pela 2ª diferença exata das poses da TF (`omniscient_accel_window: 0`)
  ou por ajuste quadrático numa janela (`> 0`).
- **Controle** (`target_accel.*`): com `enabled`, `PursuitState::a_target` entra na predição do alvo
  das leis que predizem (MPC LTV, MPC dinâmico, Hermite) com o perfil de Singer
  (`p(t) = v t + a(τt − τ²(1 − e^{−t/τ}))`), na extrapolação durante COASTING e, opcionalmente, como
  avanço `a·lead` na velocidade das demais (LQR, LQI, PID, ASMC, MPC-PX4). `lead` padrão = 0 (abaixo).
- GCS: aba Controle → *Aceleração do alvo* e *Filtro do alvo*.
- Testes: estimativa de aceleração constante, entrada do referencial, predição numa curva (Singer
  < 0,6 × CV no erro de predição a 0,5 s).

## Rodada SITL (2026-11-06, 1 repetição)

`ab_runner.py`: métricas pela verdade (TF), em regime (depois do primeiro instante a ±1 m da
distância de perseguição). `range` = RMS de |distância − 5 m|; `az`/`el` = RMS do ângulo do alvo na
imagem; `behind` = fração do tempo com o alvo atrás da câmera. Sem → com aceleração (Singer na visão).

| Cenário | Lei (uso) | range [m] | az [°] | el [°] |
|---|---|---|---|---|
| oito, omnisciente | MPC LTV (predição) | 0,40 → **0,29** | 8,0 → 8,1 | 11,1 → 16,7 |
| zigue-zague, omnisciente | MPC LTV (predição) | 0,24 → **0,16** | 4,2 → 4,2 | 4,2 → 3,8 |
| zigue-zague, visão | MPC LTV (predição) | 0,48 → **0,25** | 6,9 → **4,2** | 6,5 → 6,0 |
| oito, omnisciente | MPC-PX4 (avanço 0,25 s) | 0,34 → 2,21 | 2,4 → 10,5 | 5,4 → 15,6 |
| zigue-zague, omnisciente | MPC-PX4 (avanço) | 0,15 → 0,33 | 2,0 → 5,3 | 3,3 → 7,5 |
| zigue-zague, visão | MPC-PX4 (avanço) | 0,39 → 4,18 | 7,1 → 21,1 | 8,4 → 19,3 |
| oito, omnisciente | ASMC (avanço) | 0,32 → 0,92 | 2,3 → 4,3 | 8,5 → 9,5 |
| zigue-zague, omnisciente | ASMC (avanço) | 0,19 → 0,19 | 1,9 → 3,1 | 4,5 → 8,5 |
| zigue-zague, visão | ASMC (avanço) | 0,39 → 29,7 (perdeu) | 5,7 → 77,7 | 4,8 → 8,2 |

O oito com visão não entra na tabela: em 3 das 4 rodadas sem/ com aceleração o rastreador derivou para
o fundo e o perseguidor fugiu (alvo atrás 58–76 % do tempo), o que é o problema do
[estudo de rastreamento](../estudo_rastreamento/README.md), não do filtro.

Ruído medido da aceleração omnisciente (2ª diferença, zigue-zague com trechos retos, aceleração real
nula fora das quinas): mediana |a| = 1,1 m/s², p90 = 1,8 m/s², diferença entre amostras 0,64 m/s² RMS;
intervalo entre carimbos da TF de 20 a 44 ms.

## Conclusões (preliminares: 1 repetição)

1. **Na predição, ajuda**: o MPC LTV melhorou nos três casos comparáveis (−28 % a −48 % no erro de
   distância; az de 6,9° para 4,2° com visão). Faz sentido: o MPC prevê o alvo por 1 s ou mais e a
   aceleração entra integrada (duas vezes), o que filtra o ruído.
2. **Como avanço de velocidade, atrapalha**: MPC-PX4 e ASMC pioraram, às vezes muito. O avanço repassa
   o ruído da aceleração direto ao comando. Por isso `target_accel.lead` = 0 por padrão: ligar
   `target_accel.enabled` só afeta as leis com predição.
3. A elevação piorou no oito omnisciente com o MPC LTV (11 → 17°): a aceleração vertical estimada é
   pequena e ruidosa; vale testar só a parte horizontal.
4. Para usar a aceleração sem ruído no omnisciente, `omniscient_accel_window` (~0,2–0,3 s); o padrão
   continua sendo a diferença exata, sem janela.

**Recomendação**: disponível em todas as branches (está no código comum), ligada só com controladores
preditivos (`controller_type: mpc` com `mpc.online: true`, `mpc.mode: dynamic` ou `hermite`), junto
com `kf_model: singer` na visão. Antes de afirmar no TCC: ≥ 5 repetições por caso (o
`scenarios/studies/` + `run_study.py` já fazem isso) e o oito com visão depois do estudo de rastreamento.
