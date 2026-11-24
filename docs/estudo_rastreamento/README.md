# Estudo: robustez do rastreamento visual (deriva do rastreador)

Branch: `study/tracker-robustness`. Estudo separado da opção A; nada aqui muda o padrão da `main`
enquanto os resultados não forem conclusivos (tudo é selecionável por parâmetro).

## 1. Problema observado

`scenarios/tuning/s3_figure8.yaml` com visão (2026-11-06): o alvo começa a ~19 m, além do alcance da
câmera de profundidade (19,1 m), então o supervisor entra em **APPROACH** (linha de visada sem
distância). O perseguidor passou pelo alvo e **continuou voando para longe com o alvo atrás da câmera**
(azimute verdadeiro ≈ 180°) por 30 s, até 52 m. O TrackerVit seguiu com *score* > 0,35 em outra coisa
(fundo), o rastreamento continuou "confirmado" e o supervisor nunca entrou em LOST. Na mesma rodada
o APPROACH ainda **acelera** quando a ROI não cresce (`fallback.boost_*`), o que é exatamente o que
acontece com um ponto do fundo.

Na comparação da aceleração do alvo (opção do filtro de Kalman, `docs/estudo_aceleracao_alvo.md`) o
mesmo efeito dominou as rodadas com visão: alvo atrás da câmera em 58–76 % do tempo em 3 de 4 rodadas.

## 2. O que a literatura faz

- **Confiança + re-detecção**: rastreadores de longo prazo verificam a confiabilidade da resposta e só
  disparam uma busca global (re-detecção) quando ela cai. SiamAD usa um discriminador hierárquico
  ([Sensors 2022](https://www.mdpi.com/1424-8220/22/10/3701)); SiamSTA alterna busca local e re-detecção
  com uma política ciente do estado (1º lugar no 2º Anti-UAV Challenge,
  [ICCVW 2021](https://openaccess.thecvf.com/content/ICCV2021W/AntiUAV/papers/Huang_SiamSTA_Spatio-Temporal_Attention_Based_Siamese_Tracker_for_Tracking_UAVs_ICCVW_2021_paper.pdf));
  FSTC-DiMP guia a re-detecção por consistência espaço-temporal e cita a deriva por acúmulo de fundo no
  modelo ([Remote Sensing 2025](https://mdpi.com/2072-4292/17/16/2902)); FocusTrack ajusta a região
  de busca em vez de buscar globalmente ([arXiv 2504.13604](https://arxiv.org/pdf/2504.13604)).
- **Medidas de confiança**: APCE (*average peak-to-correlation energy*) e pico máximo da resposta
  ([LADCF + re-detecção](https://researchsquare.com/article/rs-51036/v3)); verificação por rede
  separada (MBMD, vencedor VOT2018 LT) e teste estatístico sequencial de falha
  ([arXiv 2602.12983](https://arxiv.org/pdf/2602.12983)); consistência *forward-backward*.
- **Avaliação de longo prazo**: precisão/revocação de presença do alvo, não só IoU
  ([VOT LT, arXiv 1906.08675](https://arxiv.org/pdf/1906.08675)).

Observação: quase todo o material anti-UAV é com câmera fixa no solo; no ar-ar a câmera se move e o
fundo muda o tempo todo, então a informação do próprio voo (profundidade, ego-movimento) é uma vantagem
que esses trabalhos não usam.

## 3. O que foi implementado (tudo selecionável)

### 3.1 Modelos genéricos (sem classe: o alvo é o que estiver na ROI inicial)

`/vision_tracker_node tracker_model` (trocável em execução; `models/download_models.sh`):

| Modelo | Tipo | Tamanho | Fonte |
|---|---|---|---|
| `vit` (padrão) | ViTTrack (transformer) | 0,7 MB | OpenCV Zoo |
| `nano` | NanoTrack v2 (siamesa leve) | 1,8 MB | HonglinChu/SiamTrackers |
| `dasiamrpn` | DaSiamRPN (siamesa RPN, treinada com distratores) | 160 MB | amostras do OpenCV (`--dasiamrpn`) |

### 3.2 Verificador de consistência física (`verifier.*`, `perception/track_verifier.*`)

Independente do modelo. Cada quadro confirmado passa por:

| Teste | Ideia | Parâmetros |
|---|---|---|
| `depth_check` | O alvo medido a r há pouco não pode estar além de r + v_max·Δt; em especial, não pode "pular" para além do alcance da profundidade se r estava bem dentro dele (a caixa foi para o fundo) | `memory`, `max_closing_speed`, `range_margin` |
| `scale_check` | Tamanho angular da ROI × tamanho físico aprendido enquanto havia profundidade × faixa de distância possível | `scale_ratio` |
| `appearance_check` | Centro da ROI parecido com o modelo inicial: histograma H-S (Bhattacharyya) e energia de bordas (céu/grama perdem uma das duas) | `appearance_min` |
| `score_check` (desligado) | *Score* abaixo de uma fração da própria média (tipo APCE, relativo à escala do modelo) | `score_drop_ratio` |

Quadro inconsistente: nada sai do nó (sem ponto 3D, sem linha de visada). `fail_frames` quadros
seguidos: o rastreamento é descartado (`tracker->stop`) e a aquisição automática pode readquirir.
O veredito e a aparência vão em `TargetDetection.verifier` / `.appearance` (gravados e visíveis na GCS).

### 3.3 Métricas novas (`session_info.yaml`, CSV do `run_study.py`)

- `false_track_ratio`: fração dos quadros com rastreamento **confirmado fora do alvo**, inclusive com o
  alvo fora da imagem (o caso da fuga, que antes não era contado);
- `max_target_distance`: maior distância perseguidor–alvo (fuga).

## 4. Método

`python3 ros2_ws/run_study.py scenarios/studies/estudo_rastreamento.yaml` (headless): 2 cenários ×
6 variantes (3 modelos × verificador desligado/ligado), 1 repetição nesta rodada reduzida.
Controle: os ajustes lembrados da GCS (ASMC, `vertical_mode: center_image`).

## 5. Resultados (2026-11-07, 1 repetição; `resultados_2026-11-07.csv`)

`on_target` = fração dos quadros com o alvo visível em que o rastreador está nele; `falso` =
`false_track_ratio` (confirmado fora do alvo, inclusive com o alvo fora da imagem); `LOST`/`APPROACH`
= tempo nesses modos [s]; `rmse` = erro de posição do controle.

| Cenário | Variante | on_target | IoU | falso | aquisições | LOST [s] | APPROACH [s] | rmse [m] |
|---|---|---|---|---|---|---|---|---|
| oito (s3) | vit | 0,99 | 0,72 | 0,01 | 5 | 0 | 7,2 | 0,83 |
| oito (s3) | vit + verificador | 0,98 | 0,74 | **0,00** | 4 | 20,6 | 6,7 | 1,38 |
| oito (s3) | nano | 0,61 | 0,48 | 0,80 | 1 | 0 | 42,4 | 12,99 |
| oito (s3) | nano + verificador | **0,98** | **0,77** | **0,00** | 7 | 21,2 | 6,9 | **0,90** |
| oito (s3) | dasiamrpn | 0,50 | 0,36 | 0,72 | 8 | 0,5 | 16,1 | 3,60 |
| oito (s3) | dasiamrpn + verificador | 0,80 | 0,43 | 0,13 | 9 | 15,7 | 6,8 | **0,38** |
| enxurrada (flood) | vit | 0,95 | 0,59 | 0,46 | 5 | 19,3 | 88,2 | 7,90 |
| enxurrada (flood) | vit + verificador | **0,99** | **0,73** | **0,00** | 6 | 3,2 | 5,2 | **0,90** |
| enxurrada (flood) | nano | 0,25 | 0,18 | 0,94 | 1 | 0,2 | 118,1 | 10,42 |
| enxurrada (flood) | nano + verificador | **0,99** | **0,73** | **0,00** | 12 | 5,1 | 6,0 | **0,69** |
| enxurrada (flood) | dasiamrpn | 0,93 | 0,43 | 0,03 | 11 | 22,6 | 8,7 | 1,06 |
| enxurrada (flood) | dasiamrpn + verificador | 0,94 | 0,41 | 0,03 | 9 | 2,6 | 4,6 | 0,64 |

Leitura:

1. **O verificador elimina a deriva para o fundo** em todos os modelos: `falso` caiu de 0,46–0,94 para
   0,00 (ViT, Nano) e de 0,72 para 0,13 (DaSiamRPN no oito). Sem ele, Nano ficou 42 s e 118 s em
   APPROACH "perseguindo" o fundo; com ele, ~6 s.
2. **Custo: tempo em LOST**. Cada falha descarta o rastreamento e espera a readquisição; no oito, ViT
   e Nano com verificador passaram ~21 s em LOST (contra 0 s sem), e o rmse do ViT subiu de 0,83 para
   1,38 m. Parte das falhas pode ser falsa (aparência abaixo de 0,35 no próprio alvo); a calibração
   (abaixo) mostra margem, mas a liminar e `fail_frames` precisam de varredura.
3. **Modelos**: sem verificação, o ViT (padrão atual) foi o mais estável; o NanoTrack v2 deriva com
   facilidade, mas **com o verificador empata com o ViT** sendo mais leve. O DaSiamRPN (160 MB) teve
   IoU baixo (caixa larga) e trocas de identidade com os distratores (6 no flood), que o verificador
   não pega: aparência e escala de um X3 próximo são parecidas o bastante. Identidade continua sendo o
   assunto da proposta H (porta do filtro).
4. A fuga relatada não se repetiu nesta rodada com o ViT puro (`falso` 0,01 no oito): ela é
   intermitente, o que reforça a necessidade de repetições.

Calibração em imagens renderizadas (piloto, oito com verificador): no alvo, aparência mediana 0,76
(p10 0,61) no ViT e 0,74 (p10 0,59) no Nano; score 0,56–0,78 (ViT) e 0,69–0,78 (Nano).

Conclusão preliminar: **ligar `verifier.enabled`** resolve a deriva sem depender do modelo; a próxima
rodada deve reduzir o tempo em LOST (limiar de aparência, `fail_frames`, readquisição mais rápida) e
ter ≥ 5 repetições.

## 5b. Verificador ligado por padrão, teste de caixa e suspensão (2026-11-16)

Sessão `20261116_092738_mpc_ltv` (ibvs_mpc, seleção manual, verificador **desligado**): o X4 ficou
33 s de 190 s fora da imagem com o rastreador em CONFIRMED. Assinaturas: (a) a caixa do ViT cresce até
~1500–1900 px (imagem 1920×1080) com score ~0,4; (b) quando o X4 sai da imagem, o ViT pula para o
Tiltrotor (5,0 → 6,8 m, tamanho aprendido 0,73 m contra 0,48 m: razão 1,5 < `scale_ratio`).

Mudanças:
- `verifier.enabled: true` por padrão;
- `box_check`: ROI com largura/altura acima de `max_box_fraction` (0,5) da imagem; pega (a) antes de
  qualquer tamanho aprendido;
- suspensão (`hold_time` 3 s, `recover_frames` 5): após `fail_frames`, não para o rastreador. Nada é
  publicado nem aprendido (tamanho congelado) até 5 quadros consistentes seguidos; sem recuperar
  em `hold_time`, `tracker->stop()` como antes (`hold_time: 0` = comportamento de 07/11). Motivo: com
  seleção manual não há readquisição, e o ViT voltou sozinho ao X4 duas vezes no bag.

Replay offline (profundidade, escala e caixa; sem aparência, que precisa das imagens;
`/tmp/.../replay.py`, não versionado):

| Sessão | quadros errados publicados (sem → com) | bons suprimidos | descartes (`hold` 3 s / 0 s) |
|---|---|---|---|
| `092738_mpc_ltv` | 817 → 566 | 8 de 4458 | 5 / 83 |
| `093259_chained_laps_r1` | 303 → 0 | 0 de 5592 | 4 |

Os 566 restantes são o caso (b), o Tiltrotor e o fundo ao redor. O replay é pessimista: após um
descarte, o rastreador gravado continua e o replay reaprende o objeto errado; no nó, o rastreador para.
`scale_ratio` 1,4 ou 1,3 quase não ajuda (566 → 415 → 365) e suprime 3× mais quadros bons: fica 2,0.
A troca de identidade continua aberta e pede a porta da linha de visada no filtro (abaixo).

## 6. Próximos passos

- Incongruência cinemática no APPROACH: voando na direção da linha de visada, a ROI de um alvo real
  cresce (tempo até o contato finito); a de um ponto do fundo não. Hoje o APPROACH faz o contrário
  (acelera). Teste: distância voada ao longo da linha de visada × crescimento da ROI.
- Porta da linha de visada no filtro (ângulo entre a direção medida e a prevista pela última estimativa
  3D), além da porta χ² que já existe para as medidas 3D.
- Verificador de aparência por embedding genérico (CLIP ViT-B/32, já em `weights/clip`, roda na GPU
  ROCm desta máquina), a poucos Hz, para distinguir também distratores parecidos.
- Re-detecção sem *ground truth*: hoje a readquisição automática do simulador usa a projeção verdadeira;
  num voo real seria preciso uma busca global (o próprio rastreador numa janela maior, ou um detector).
