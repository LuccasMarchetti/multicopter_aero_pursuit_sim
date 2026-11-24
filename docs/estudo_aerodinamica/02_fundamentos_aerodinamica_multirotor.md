# 2. Aerodinâmica de multirrotores: o que importa e quanto

Referências completas em [referencias.md](referencias.md). Números do x500 em
`scripts/ordens_de_grandeza.out.txt`.

## 2.1 Decomposição usada na literatura

Quase todos os simuladores e trabalhos de identificação separam o wrench aerodinâmico em duas
partes. Assim fazem RotorPy, ArduPilot SITL, o EKF2 do PX4, Faessler et al. 2018, Svacha et al.,
Hattenberger et al. 2023 e Bauersfeld et al. 2021:

```
F_aero = F_rotores(ω_i, v_ar,i)  +  F_corpo(v_ar, α, β)
M_aero = Σ r_i × F_rotor,i + M_rotores (flapping, reação)  +  M_corpo(α, β, p, q, r)
```

### A) Efeitos dos rotores (dependem de ω e da velocidade no plano do disco)

| Efeito | Forma usual | Ordem de grandeza / quando importa |
|---|---|---|
| **Rotor drag** (H-force + arrasto induzido, *momentum drag*) | **F** ≈ −ω k_d **v**⊥ (linear em v e em ω) | Domina abaixo de ~10 m/s. Já existe no Gazebo |
| **Blade flapping** | inclina o vetor de empuxo para trás: força ∝ ω v⊥ (forma igual ao rotor drag, por isso é somado a ele) e momento M ∝ ω v⊥ | Momento de arfagem não desprezível acima de ~5 m/s. No Gazebo: c_r = 10⁻⁶, 31 mN·m a 10 m/s |
| **Perda de empuxo com inflow axial** (subida, vento vertical) | ΔT ≈ −k_z ω v_z (RotorPy "induced inflow coeff") | Subidas rápidas, vento vertical |
| **Sustentação translacional** | ΔT ≈ +k_h (v_x² + v_y²) | Ganho de empuxo em voo à frente; RotorPy tem o termo, mas o README admite que ainda não é fiel |
| **Empuxo × razão de avanço e ângulo do disco** | BEMT / momentum: T(ω, V, α_disco) | Em alta velocidade, T = k_f ω² erra muito. Gill & D'Andrea 2019 dão modelos paramétricos baratos |
| **Efeito solo** | T_ge/T = 1/(1 − (R/4z)²) (Cheeseman–Bennett), ou o ajuste de Shi et al. usado no gym-pybullet-drones | Só abaixo de ~2 diâmetros de hélice. Afeta decolagem e pouso |
| **Vortex ring state** | região crítica de descida em −0,2 … −1,5 v_h. Momentum theory não vale; o modelo da ONERA é o mais próximo do experimento (Talaeizadeh et al.) | Descida vertical rápida: relevante para os **modos verticais** do branch atual |
| **Interação rotor–rotor e rotor–corpo** | perda de ~5 % do empuxo a 20 ft/s (NASA, SOLO) | Pequena; vai como fator de correção |

### B) Efeitos do corpo (airframe, bluff body)

| Efeito | Forma usual |
|---|---|
| **Arrasto parasita** | **F** = −½ ρ \|v\| CdA(α, β) **v**, quadrático |
| **Sustentação e força lateral do corpo** | ½ρV² S C_L(α), C_Y(β). Desprezível em quadricópteros pequenos ("all generate very little lift", NASA) e significativa em corpos tipo disco (DAx8) |
| **Momentos do corpo** | ½ρV² S l C_{l,m,n}(α, β), mais amortecimento de rotação C_{mq}, … |

## 2.2 Dados medidos que dão para usar direto

**NASA, Russell et al. (2016/2018), túnel 7×10 ft da Ames.** Cinco multirrotores comerciais
(DJI Phantom 3, 3DR Iris+, 3DR SOLO, DAx8, SUI Endurance), pitch de −40° a +40°, 20 e 40 ft/s,
com e sem rotores, além de rotor isolado. Da Fig. 30 (airframe sem rotores):

- Drag/q ≈ **0,18–0,45 ft² ≈ 0,017–0,042 m²** para Iris+, SOLO e Phantom. Curva em "U" com mínimo
  perto de α ≈ 0°, chegando a ~1,5–2× em |α| = 40°;
- Lift/q ≈ 0 nos quadricópteros (o DAx8, em forma de disco, gera sustentação considerável);
- efeito de Reynolds: a velocidade menor dá Drag/q maior nos quadricópteros pequenos;
- com rotores, o arrasto total cresce monotonicamente com α (os rotores puxam para frente ou
  para trás) e o trim do SOLO fica em α ≈ −6°;
- o rotor isolado tem momento de rolagem no cubo (lift offset) de 5–8 % do raio, que varia mais com
  o pitch do eixo do que com a rotação.

O Iris+ (550 mm, ~1,3 kg) é o análogo publicado mais próximo do x500 (610 mm, ~2 kg). Como
primeira estimativa, CdA_x ≈ 0,03 m² é defensável, com faixa de 0,017 a 0,042.

**ArduPilot SITL** calibra o arrasto do corpo por um único ponto (velocidade de referência e
inclinação de referência, padrão 15,08 m/s a 45°). Divide a força entre momentum drag (pela área do
disco) e bluff body: CdA = (m g tan θ_ref − D_momentum)/(½ ρ V_ref²). É a forma mais simples de
amarrar o modelo a um dado de voo real: basta um voo em velocidade constante, com inclinação medida.

**Hattenberger, Bronz & Condomines (ENAC, IJMAV 2023).** Coeficiente de arrasto de um quadricóptero
medido com gerador de vento e captura de movimento, mais um método de mínimos quadrados a partir de
voo externo. Também mostra a dependência com massa e com a forma da fuselagem.

**Faessler, Franchi & Scaramuzza (RA-L 2018).** Rotor drag linear (D = diag(d_x, d_y, d_z)) mantém a
planeza diferencial e reduz o erro de rastreamento em cerca de 50 % em alta velocidade. Os
coeficientes são identificados por otimização sem gradiente. Isso se conecta direto com
`control/flatness.hpp` do projeto.

**Bauersfeld et al., NeuroBEM (RSS 2021).** BEM (blade element momentum) mais uma rede neural para
o resíduo. Medido até 65 km/h e com erro ~50 % menor que os modelos anteriores. É o estado da arte
de fidelidade, mas pesado e dependente de dados de captura de movimento.

**Sun, de Visser et al. (TU Delft).** Identificação *gray-box* em voo de alta velocidade e em túnel
grande (α de −90° a 90°, vento de 0 a 14 m/s). Coeficientes de empuxo e torque tabelados por razão
de avanço e ângulo de ataque. É exatamente o formato de tabela sugerido nesta pesquisa.

## 2.3 "Arrasto de placa plana" funciona?

Funciona como **ordem zero**, com cuidados:

- Placa plana normal ao escoamento: C_D ≈ 1,17–1,28. Para corpos rombudos 3D, 0,8–1,2. A soma das
  áreas projetadas por eixo (frente, lado, topo) com C_D ≈ 1 dá CdA na faixa certa (0,02–0,04 m²
  frontal no x500). Mas **não captura a curva em "U" com α** se usar só os três eixos isoladamente.
- O AirSim usa exatamente uma variante disso: soma, por face de uma caixa, F = −k ρ v_n² **n**
  (só faces "a barlavento"). Isso gera de graça a dependência com α e β e momentos (r × F), mas
  superestima o arrasto em ângulos oblíquos e ignora a sombra entre componentes.
- A placa plana **inclinada** também gera força normal, logo "sustentação" (C_N ≈ C_D,90 sin²α a
  grandes ângulos, Hoerner). Num corpo com chapas superior e inferior largas, como o x500 (placas de
  ~0,35 × 0,35 m), isso não é desprezível em α grande. A NASA mostra Lift/q ≈ 0 para quadricópteros
  *pequenos*, o que limita esse efeito em magnitude.
- A melhor forma de usar a ideia é **offline**: decompor a malha (ou a geometria simplificada do
  SDF) em painéis, calcular a força por painel (Newtoniano/placa plana) para uma grade de (α, β) e
  gravar CdA_ij e os momentos numa tabela. Isso dá a forma da curva. Depois se escala tudo por **um**
  fator para bater com um ponto de referência (NASA Iris+, ponto de voo estilo ArduPilot, ou CFD).

## 2.4 Prioridade física para o nosso problema (perseguição ar-ar)

1. **Arrasto parasita do corpo** (quadrático, com α e β): maior lacuna e muda velocidade máxima,
   frenagem e inclinação de cruzeiro.
2. **Vento com turbulência e rajadas** (Dryden/von Kármán ou campo pré-gerado) aplicado de forma
   coerente a corpo e rotores. É o que perturba visão e controle na prática.
3. **Momentos do corpo e amortecimento de rotação**: afetam a atitude em alta velocidade e o
   apontamento da câmera.
4. **Variação do empuxo com a velocidade** (inflow, razão de avanço): afeta o empuxo de cruzeiro e as
   subidas.
5. **Esteira ou downwash do alvo**: o perseguidor voa atrás e às vezes abaixo do alvo. Kiran et al.
   (RSS 2025) e o estudo de 16 h de voo (arXiv 2403.13321) mostram efeitos mensuráveis. Para esta
   aplicação é específico e diferenciador, mas de 2ª ordem.
6. **Efeito solo e VRS**: só para decolagem, pouso e descida rápida (modos verticais).
