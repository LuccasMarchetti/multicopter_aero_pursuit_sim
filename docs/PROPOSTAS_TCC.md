# Propostas de escopo para o TCC

Data: 2026-11-05. Base: o estado de `main` (commit 813bc46) e uma pesquisa de TCCs da EESC/USP.

## 0. Diagnóstico

### O que a plataforma já tem

| Bloco | Já existe |
|---|---|
| Simulação | PX4 SITL + Gazebo Harmonic, x500_depth, alvos X3/X4/Tiltrotor, vento e neblina, plugin aerodinâmico `aero_gz_physics` com tabela corrigida por CFD (erro de 7,3%) |
| Percepção | TrackerVit + profundidade, ROI calibrada, fallback por tamanho da ROI, modo omnisciente exato (ground truth) |
| Estimação | KF do alvo, com ruído separado para a fonte omnisciente |
| Controle | MPC/LTV-MPC (cinemático e dinâmico, com saídas attitude, thrust_torque e motors), LQR, LQI, PID-PBVS, ASMC, Hermite/planeza diferencial, PN/PP híbrido, política externa (RL) |
| Supervisão | TRACKING / COASTING / APPROACH / LOST, banda morta, políticas de yaw e do canal vertical |
| Experimentos | cenários YAML reprodutíveis, enxames distratores, `run_study.py` (cenário × variante × repetições), `/pursuit/metrics` (RMSE de posição, velocidade e LOS, esforço ∫\|a\|², tempo de cálculo), MCAP por engajamento |
| Texto | `guia_controle/`, um guia de 55 páginas com a teoria de cada lei, a metodologia de comparação (custo comum, estatística) e um estudo de caso |

### Referência de escopo (EESC, mesmo orientador)

- **Riascos, J. *Controle de Drones com Modelo Baseado em Quatérnios*, EESC/USP, 2024, 67 p.** A banca foi presidida pelo Prof. Glauco Caurin. O trabalho compara **dois** controladores (LQR com modelo de Euler-Lagrange e realimentação de estados com quatérnios) **só em MATLAB**. Estrutura: Introdução → Modelos → Controladores → Simulação → Resultados → Conclusões.
- **Azevedo, L. F. S.: piloto automático de VANT de asa fixa para sensoriamento de gases**, orientado por Caurin. É um trabalho de integração de sistemas (sensores, Pixhawk, voo de teste no campus).
- Fora da USP há TCCs de graduação com 2 a 4 controladores comparados em simulação (UFC 2025: DeePC × PID × LQR × MPC; UFU: rastreamento visual com PID).

**Conclusão:** um TCC de graduação nesse grupo cabe em **uma pergunta, 2 a 4 métodos e uma bateria de simulações bem analisada**. A plataforma já supera em muito o que o texto precisa. O risco não é faltar conteúdo, e sim **não ter foco** e tentar documentar tudo. A escolha do escopo é, na prática, escolher **qual fatia** da plataforma vira o objeto de estudo. O resto entra como "ambiente experimental" (um capítulo de metodologia).

### Estrutura comum a todas as opções

1. Introdução: motivação (contra-UAS, perseguição ar-ar), problema, objetivos, contribuição.
2. Revisão: perseguição e servovisão (IBVS/PBVS), as leis do escopo e trabalhos correlatos (`source/`).
3. Modelagem: quadrirrotor, laço do PX4 como planta equivalente, cinemática relativa perseguidor-alvo.
4. Métodos: projeto das leis escolhidas.
5. Ambiente experimental: arquitetura ROS 2/PX4/Gazebo, percepção, KF, cenários e métricas (resumo, não manual).
6. Resultados: experimentos da opção.
7. Conclusões e trabalhos futuros.

Meta: 60 a 90 páginas. O que não entra no escopo vira "trabalhos futuros", o que também mostra ao avaliador que a plataforma é maior que o TCC.

---

## Opção A: Comparação de leis de controle para perseguição de alvo manobrante

**Pergunta.** Para um alvo que manobra, quanto ganha cada lei com o uso da predição do movimento do alvo, e a que custo de esforço e de cálculo?

**Escopo (3 a 4 leis, uma por família):**
- PID-PBVS com feedforward (clássica, linha de base)
- LQI (ótima, sem horizonte)
- LTV-MPC (ótima, com predição e restrições)
- opcional: guiamento PN/PP (família da aeronáutica, sem modelo)

**Fora do escopo:** visão (usar o modo omnisciente para **isolar o controle**), MPC dinâmico, RL, enxames.

**Experimentos**
1. Sintonia justa: todas as leis ajustadas para o mesmo custo J = ∫(eᵀQe + ρ aᵀa) dt (cap. 13 do guia), ou por busca em grade com o `run_study.py`.
2. Cenários com dificuldade crescente: alvo parado (regime), velocidade constante, oito, zigue-zague evasivo, degrau de velocidade.
3. Monte Carlo: N ≥ 10 repetições por par (lei × cenário), com média, desvio e intervalo de confiança, e teste de Wilcoxon ou ANOVA entre as leis.
4. Robustez: vento constante e rajadas (LQI × PID mostram o efeito da ação integral), variação de `max_acc`.

**Métricas:** RMSE de posição ao ponto de standoff, erro de distância, erro de LOS, esforço ∫|a|², pico de inclinação, tempo de cálculo por ciclo e percentual de tempo em cada modo do supervisor.

**Figuras-chave:** trajetórias 3D sobrepostas, erro × tempo, gráfico de Pareto desempenho × esforço (uma das melhores figuras para a banca), boxplots por cenário, tempo de cálculo × horizonte N do MPC.

**Estado:** cerca de 85% da implementação existe. Falta automatizar a sintonia justa, rodar o lote e fazer a estatística.
**Esforço:** baixo a médio. **Risco:** baixo. **Ponto fraco:** é o tema "mais comum". A contribuição depende do rigor da sintonia justa e da estatística, que precisam ser bem feitas.

---

## Opção B: Perseguição visual com percepção degradada (atraso, ruído e perda de profundidade) ★ recomendada

**Pergunta.** Como o atraso e as falhas da percepção embarcada degradam a perseguição, e uma compensação simples do atraso (predição do estado) recupera o desempenho?

Esta é a opção que melhor junta controle com o seu lado de eletrônica e sistemas embarcados: latência de cadeia, taxa de amostragem e sensores. A plataforma já mede tudo isso (cap. 12c do guia).

**Escopo**
- 2 a 3 leis (ex.: LQI e LTV-MPC, e PID como base)
- 3 fatores de degradação, injetados de forma controlada sobre a medida omnisciente: **atraso** (0 a 400 ms), **ruído** (σ em função da distância, como na câmera de profundidade) e **perdas** (quadros sem detecção, profundidade ausente → APPROACH/COASTING)
- 1 contribuição: **compensação de atraso**, que prediz o estado atual a partir da medida atrasada (preditor de Smith ou propagação do KF até t + τ). Essa pendência já está listada em `PENDENCIAS.md`.
- validação final com a **visão real** (TrackerVit), mostrando que a degradação sintética representa a real

**Experimentos**
1. Caracterização da cadeia real: latência imagem → setpoint, jitter e taxa de detecção (dados já gravados em MCAP).
2. Varredura de atraso: margem até a instabilidade de cada lei, com uma análise teórica da margem de fase do laço linearizado (PX4 de 1ª/2ª ordem + atraso) comparada ao limite medido. É a parte "de controle" forte do trabalho.
3. Varredura de ruído e de perdas: efeito no RMSE e no esforço, e papel do KF e do coasting.
4. Com e sem a compensação: ganho de desempenho para cada lei.
5. Visão real × omnisciente degradado com os mesmos parâmetros, para validar o modelo de degradação.

**Métricas:** as da opção A, mais o atraso crítico τ*, a taxa de perda do alvo (LOST), o tempo em COASTING e a recuperação.

**Figuras-chave:** RMSE × atraso (uma curva por lei, com e sem compensação), mapa de estabilidade (atraso × ganho), diagrama de Nyquist/Bode com atraso, histograma de latência medida.

**Estado:** leis, KF, supervisor, omnisciente e métricas existem. Falta o **injetor de degradação** (um nó pequeno ou parâmetro no estimador), a **compensação** e a análise de margem.
**Esforço:** médio. **Risco:** baixo a médio. **Ponto forte:** tem uma pergunta clara, uma contribuição própria (a compensação), teoria (margens) confrontada com simulação, e é diferente de "comparei N controladores".

---

## Opção C: Até onde descer na cascata: MPC cinemático × dinâmico (velocidade, atitude, empuxo/torque, motores)

**Pergunta.** Vale a pena o controlador de perseguição contornar os laços internos do PX4? Qual é o ganho de banda e o custo em robustez a erro de modelo?

**Escopo:** um único MPC em 2 a 4 níveis de saída (`trajectory`, `attitude`, `thrust_torque`, `motors`). Modelagem do x500 (massa, inércia, coeficientes dos rotores, alocação) e identificação do laço de atitude do PX4 (2ª ordem, ωn ≈ 11, ζ ≈ 0,9).

**Experimentos:** resposta ao degrau e banda de cada nível; perseguição de alvo agressivo; erro de modelo (±20% em massa e inércia, coeficiente de empuxo); atraso; tempo de cálculo do QP por nível.

**Estado:** implementado, com resultados preliminares (zigue-zague com σ de 0,15 m no dinâmico contra 0,04 m no cinemático; altitude oscilando ±1 m em thrust_torque/motors). Falta: compensar o atraso de ~0,2 s, integrar o erro vertical e investigar a queda na troca de saída.
**Esforço:** médio a alto. **Risco:** médio, porque os resultados atuais são **desfavoráveis** ao dinâmico. Isso é válido ("a cascata do PX4 já é boa"), mas exige explicar a causa com cuidado.
**Ponto forte:** é muito coerente com engenharia aeronáutica (dinâmica de voo + controle) e com a linha de Riascos (modelagem → controlador). É a opção mais "de controle" no sentido clássico.

---

## Opção D: Guiamento de interceptação aplicado a multirrotores (PN/PP × controle ótimo)

**Pergunta.** As leis de guiamento de mísseis (navegação proporcional, perseguição pura) servem para um multirrotor que precisa **manter distância** do alvo, e não colidir com ele? Como se comparam ao MPC?

**Escopo:** PP, PN (N = 3 a 5) e PN/PP híbrida com manutenção de distância, contra o LTV-MPC como referência. Cinemática de engajamento 2D/3D: taxa de LOS, velocidade de aproximação, região de captura.

**Experimentos:** região de captura (grade de posições e velocidades iniciais → captura ou falha); tempo até o standoff; efeito de N; alvo com manobra lateral (aceleração do alvo / aceleração máxima do perseguidor); análise analítica 2D comparada à simulação.

**Estado:** a lei `hybrid_pn` existe (cap. 12 do guia). Falta a varredura da região de captura e o modelo analítico.
**Esforço:** baixo a médio. **Risco:** baixo. **Ponto forte:** tem forte identidade aeronáutica e figuras muito didáticas (mapas de captura). **Ponto fraco:** é o tema mais distante de "técnicas de controle" em sentido estrito.

---

## Opção E: Influência do modelo aerodinâmico (arrasto via CFD) no controle de perseguição em alta velocidade

**Pergunta.** O arrasto, ignorado pela simulação padrão e pelos modelos dos controladores, muda o desempenho da perseguição em alta velocidade? Um feedforward de arrasto, a partir da tabela do CFD, compensa essa diferença?

**Escopo:** simulação padrão do Gazebo × `aero_gz_physics` com a tabela do CFD; MPC/LQI com e sem o termo de arrasto no modelo; velocidades de 2 a 12 m/s.

**Experimentos:** erro de regime em velocidade constante; perseguição em oito rápido; vento contrário e lateral; comparação com a ação integral (o LQI "absorve" o arrasto?).

**Estado:** o plugin, a tabela de CFD do x500 (10 direções) e o relatório (`docs/estudo_aerodinamica/`) existem. Falta o feedforward nos controladores e a bateria de ensaios.
**Esforço:** médio. **Risco:** médio, porque o efeito pode sair pequeno nas velocidades do x500. **Ponto forte:** é a opção mais "aeronáutica" de todas e reaproveita o trabalho de CFD, que de outra forma fica sem destino no TCC.

---

## Opções desaconselhadas como tema principal

- **RL × controle clássico:** o ambiente existe, mas treinar bem consome meses de ajuste e a comparação justa é difícil. Fica como trabalho futuro.
- **A plataforma como produto** (GCS, Docker, cenários): é válido como capítulo de metodologia, mas como tema principal parece engenharia de software, e o interesse declarado é controle.
- **Enxame de perseguidores:** é experimental e abre escopo demais.

## Recomendação

| | A | **B** | C | D | E |
|---|---|---|---|---|---|
| Foco em controle | alto | **alto** | muito alto | médio | médio |
| Identidade aeronáutica | média | média | alta | alta | muito alta |
| Ligação com eletrônica/embarcados | baixa | **alta** | média | baixa | baixa |
| Pronto hoje | ~85% | ~70% | ~60% | ~75% | ~60% |
| Risco | baixo | **baixo-médio** | médio | baixo | médio |
| Contribuição original | fraca | **clara** | clara | média | clara |

**B como tema principal, com A embutida:** o capítulo de resultados começa pela comparação das leis em condição ideal (é a opção A em versão curta) e depois degrada a percepção. Assim o TCC continua sendo sobre "técnicas de controle para perseguição", como planejado, mas com uma pergunta que vai além de uma tabela comparativa.

Título provisório: *Perseguição aérea baseada em visão: desempenho de leis de controle sob atraso e falhas de percepção e compensação por predição de estado.*

Alternativa se o orientador puxar para dinâmica de voo: **C**. Se puxar para aerodinâmica: **E**. Ambas podem entrar como capítulo extra de B só se sobrar tempo.

---

## Novas sugestões (rodada 2, 2026-11-05 noite)

Documentos-modelo curtos em abnTeX2: `latex/opcao_f/`, `latex/opcao_g/` e `latex/opcao_h/` (mesmo
preâmbulo e bibliografia da opção A).

Surgiram ao implementar a opção A (MPC-PX4, variante sem *feedforward*, sintonia em lote) e ao rodar os
primeiros ensaios SITL. Cada uma tem um **trabalho de exemplo** já feito no repositório, que serve de
prova de conceito e de primeiro resultado.

### Opção F: O autopiloto como planta: identificação da malha interna do PX4 e controle ciente dela

**Pergunta.** Quanto do desempenho de uma lei de perseguição depende de conhecer a malha interna do
autopiloto, e dá para identificá-la em operação (sem ensaio dedicado) a partir dos próprios voos?

**Escopo:** identificação de 1ª ordem + tempo morto da malha de velocidade do PX4 a partir dos logs de
perseguição (mínimos quadrados) e por degraus; MPC-PX4 com τ fixo × τ identificado × τ estimado on-line
(RLS, já implementado: `mpc.adaptive_tau`); variantes do PX4 (nominal, lenta, sem *feedforward*).
**Exemplo feito:** `ros2_ws/identify_px4_lag.py` sobre as sessões S4 do SITL. Com *feedforward*:
τ ≈ 0,20 s e tempo morto ≈ 20 ms; sem *feedforward*: τ ≈ 0,35–0,43 s e tempo morto ≈ 0,26–0,36 s. O
modelo simplificado da opção A previa 45 ms e 433 ms: acerta a ordem de grandeza sem *feedforward*,
mas subestima em 4× a malha nominal. É exatamente o erro de modelo que o MPC-PX4 embute. Com o τ identificado (200 ms) o MPC-PX4 caiu de 1,43 m para 0,50 m de RMSE em S4 (mesmos pesos, 1 ensaio); com a estimação on-line, 0,58 m.
**Esforço:** baixo a médio. **Risco:** baixo. **Ponto forte:** contribuição clara (o atraso real do
autopiloto medido em operação) e ligação direta com dinâmica de voo.

### Opção G: Sintonia automática transferível: do modelo simplificado ao SITL (e ao voo)

**Pergunta.** Ganhos ótimos num modelo barato continuam ótimos no SITL? Quantas avaliações no SITL a
otimização precisa para corrigir a diferença, e que método (Nelder–Mead, otimização bayesiana) gasta menos?

**Escopo:** o custo comum *J* da opção A, o laço `tune_study.py` (Nelder–Mead em log θ sobre o SITL) e
um otimizador bayesiano como alternativa (já implementado: `tune_study.py --method bo`); partida no ótimo do modelo × partida no padrão.
**Exemplo feito:** sintonia curta do MPC-PX4 (omnisciente, S4, 8 avaliações ≈ 13 min): a paisagem perto
do padrão do texto (q_p = 20, q_v = 4) é plana (*J* entre 3,53 e 3,80), enquanto os pesos do YAML
(10; 2) deram RMSE de 0,52 m contra ~1,8 m: o ótimo do modelo não transfere e a busca local não sai
do platô. Isso motiva o estudo.
**Esforço:** médio. **Risco:** médio (custo de SITL). **Ponto forte:** metodologia reaproveitável
por qualquer trabalho do grupo.

### Opção H: Percepção sob distratores: troca de identidade em enxames e seu efeito no controle

**Pergunta.** Com drones parecidos cruzando a linha de visada, com que frequência o rastreador troca de
alvo, e que mudanças no supervisor (*gating* do KF, reaquisição, verificação por modelo) evitam que o
controle persiga o drone errado?

**Escopo:** cenários `flood_decoys` e `decoy_swarm`, métricas de verdade já gravadas
(`tracked_identity`, `identity_switches`, IoU, tempo no alvo), duas ou três estratégias de rejeição.
**Exemplo pronto para rodar:** `scenarios/studies/sugestao_h_decoys.yaml` com `flood_decoys` × {gating padrão, gating
apertado} × 5 repetições usa só o que já existe. Ensaio reduzido (1 repetição, 120 s): gating padrão 4 trocas de identidade e RMSE 2,02 m; estreito 2 trocas e 1,13 m; desligado 0 trocas e 2,21 m.
**Esforço:** médio. **Risco:** médio. **Ponto forte:** liga percepção e controle; forte apelo prático.

### Opção I: Orçamento de latência de ponta a ponta (câmera → motor) e limite de banda da perseguição

**Pergunta.** Qual é o atraso total entre o evento e a resposta dos motores (câmera, inferência, DDS,
estimador, lei, PX4) e qual banda de perseguição ele permite, pela margem de fase?

**Escopo:** medir cada etapa (carimbos já existentes: quadro, inferência, `/pursuit/metrics`, odometria;
o tempo morto identificado na opção F), montar o orçamento e validar o limite com a varredura de ganho.
**Esforço:** baixo a médio. **Risco:** baixo. **Ponto forte:** ligação com eletrônica/embarcados e uma
regra de projeto simples (banda máxima × atraso) que o texto pode provar no SITL.

## Próximos passos (independentes da opção)

1. Levar 2 ou 3 opções ao Prof. Caurin e decidir o tema.
2. Congelar a plataforma: só correções e o que o escopo exige.
3. Criar o esqueleto LaTeX em `latex/` (modelo da EESC), reaproveitando capítulos do `guia_controle/`.
4. Definir a matriz de experimentos (cenários × variantes × repetições) como um YAML em `scenarios/studies/` e gerar as figuras por script, de forma reprodutível.
