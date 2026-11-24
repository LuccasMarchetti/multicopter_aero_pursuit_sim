# 4. Modelo pré-calculado (tabelas): formulação, fontes de dados e formato

A ideia de pré-calcular e só consultar em tempo de simulação é a forma certa de ter
não-linearidade (curva em "U" com α, acoplamento com β, momentos) com custo O(1). É o que fazem
JSBSim, as bases da NASA LaRC e a TU Delft. Este capítulo define **o que** tabelar, **como gerar**
e **como consultar**.

## 4.1 Variáveis e referenciais

Velocidade do ar no referencial do corpo (FLU do Gazebo), no ponto de referência aerodinâmico
**r**_ref (≈ centro geométrico):

```
v_ar = Rᵀ (v_CM + ω × (R r_ref) − v_vento(p, t))     V = |v_ar|
α = atan2(−w, u)         ∈ (−180°, 180°]   ângulo de ataque (FLU: w para cima; o sinal − dá α > 0 com o
                                            vento relativo vindo de baixo, como na convenção FRD aeronáutica)
β = asin(v / V)          ∈ [−90°, 90°]     derrapagem
```

Para um corpo sem simetria de revolução é preciso cobrir **toda a esfera de direções**. Um
multirrotor voa de lado, de ré e desce verticalmente (α = ±90°). Por isso a grade tem que ir a
α ∈ [−180°, 180°] e β ∈ [−90°, 90°], não só à faixa de avião.

Coeficientes 6-DOF adimensionais no referencial do corpo (forma de "força por eixo", mais robusta
que lift/drag em α grande):

```
F_corpo = q S_ref [C_X, C_Y, C_Z](α, β, V̂)          q = ½ ρ(h) V²
M_corpo = q S_ref l_ref [C_l, C_m, C_n](α, β, V̂) + amortecimento(p, q, r)
```

- S_ref: área de referência fixa (por exemplo, a área frontal ou a de topo). l_ref: semienvergadura.
- V̂ (ou Reynolds) como 3ª dimensão é opcional. A NASA mostra efeito de Reynolds pequeno, mas
  presente em quadricópteros pequenos.
- Amortecimento de rotação: termos de derivada (C_lp, C_mq, C_nr) separados em vez de tabela 6-D.
  Pode ser constante ou função de α.
- ρ(h) pela atmosfera adiabática que o mundo já declara (ISA).

## 4.2 Tabelas dos rotores (opcional, fase 2)

Mantendo o `MulticopterMotorModel` para k_f ω², reação e rotor drag, uma correção por rotor em
função da razão de avanço e do ângulo do disco:

```
μ = V_⊥ / (ω R)           λ_c = V_∥ / (ω R)          (no plano do disco / axial)
T = k_f ω² · f_T(μ, λ_c)                f_T(0,0) = 1
H = k_f ω² · f_H(μ, λ_c)                (substitui o c_d linear quando ativado)
M_flap = k_f ω² R · f_M(μ, λ_c)
```

A tabela f_T, f_H, f_M sai de BEMT (blade element momentum) com a geometria da hélice. Também sai
dos modelos paramétricos de Gill & D'Andrea (2019), ou das tabelas de C_T e C_Q por (J, α) da TU
Delft. A NASA (rotor isolado) dá dados para validar. **Cuidado:** aplicar f_T como correção sobre o
empuxo do `MulticopterMotorModel` exige que os dois sistemas leiam o mesmo ω. Ou o novo sistema
lê a velocidade da junta e soma ΔT = k_f ω² (f_T − 1) no link do rotor, ou se escreve um motor model
próprio. A primeira opção não toca no PX4.

## 4.3 Como gerar as tabelas (do mais barato ao mais caro)

| # | Fonte | Custo | O que dá | Fidelidade |
|---|---|---|---|---|
| A | **Placa plana composta por eixo** (CdA_x, CdA_y, CdA_z) com C_D ≈ 1 nas áreas projetadas do SDF/CAD | minutos | só a diagonal, sem forma em α | baixa/média, ok até ~10 m/s |
| B | **Painéis Newtonianos / placa plana por triângulo** sobre a malha visual ou o CAD do x500, com sombra por *ray casting*, em grade (α, β) | 1 script Python + trimesh; segundos a minutos por veículo | C_X..C_n completos, forma da curva, momentos | média. Bom formato, magnitude a calibrar |
| C | **Calibração por 1–3 pontos de voo real** (método ArduPilot: V e θ em regime) ou por mínimos quadrados em logs `.ulg` (Hattenberger 2023, Faessler 2018) | 1 voo | escala de B ou de A | alta na faixa voada |
| D | **Dados publicados**: NASA Russell (Iris+/SOLO, Drag/q vs α), TU Delft | leitura | curva de referência | proxy (outro veículo) |
| E | **VSPAERO/OpenVSP** (VLM/painéis) | horas | sustentação e momentos de superfícies finas | ruim para bluff body |
| F | **CFD RANS** (OpenFOAM `simpleFoam` + k-ω SST, ou SU2) sobre o airframe sem hélices, varrendo ~50–150 pares (α, β) | dias de CPU | tabela completa do corpo | alta para o corpo isolado |
| G | **DUST** (painéis + partículas de vórtice, open source) ou CFD com disco atuador | dias/semanas | interação rotor–corpo, esteira, downwash | alta, cara |
| H | **Rede neural residual** (NeuroBEM) | precisa de mocap | resíduo | estado da arte, fora do escopo |

**Recomendação:** B para a forma, mais D ou C para a escala. A tabela fica fisicamente coerente em
toda a esfera de direções e ancorada em um número medido. F fica como validação ou trabalho futuro.
No x500 isso cabe num TCC. No alvo (X3/X4/Tiltrotor) o mesmo script gera uma tabela por modelo.

## 4.4 Grade, formato e consulta

- **Grade:** α de 5° em 5° (73 pontos), β de 5° em 5° (37 pontos), V opcional em ~12 pontos.
  6 coeficientes float32 dão ~63 KiB sem V e ~760 KiB com V por veículo (cabe em L2).
- **Interpolação:** trilinear numa grade uniforme (índice direto, sem busca), O(1). Se a derivada
  importar para linearizar no MPC, usar bicúbica ou *spline* de Catmull-Rom, também O(1).
- **Periodicidade:** α é periódico (−180° ≡ 180°). Nos polos β = ±90°, α fica indefinido. Usar a
  forma de "força por eixo" (C_X, C_Y, C_Z), que é contínua lá, ou parametrizar pela direção
  unitária (u, v, w)/V numa grade *cube-map* ou de Fibonacci. Esta é uma armadilha conhecida de
  tabelas (α, β) em multirrotores que descem na vertical.
- **Velocidade baixa:** para V < ~0,5 m/s, mandar a força para zero suavemente. α e β ficam
  ruidosos, mas a força ∝ V² já é desprezível.
- **Formato:** CSV ou YAML legível (versionável no git) com cabeçalho (S_ref, l_ref, r_ref, unidades,
  origem dos dados, data), carregado uma vez no `Configure()` do plugin. Opcionalmente binário
  `.npy` para tabelas grandes. Exemplo:

```yaml
# x500_body_aero.yaml (gerado por tools/gen_aero_table.py, fonte: painéis + escala NASA Iris+)
reference: {S_ref: 0.03, l_ref: 0.25, r_ref: [0, 0, 0.03]}
grid: {alpha_deg: [-180, 180, 73], beta_deg: [-90, 90, 37]}
damping: {Clp: -0.4, Cmq: -0.4, Cnr: -0.1}
coefficients: {CX: [...], CY: [...], CZ: [...], Cl: [...], Cm: [...], Cn: [...]}
```

## 4.5 Vento

O valor da tabela depende de **v_ar**, então o vento tem que ser o mesmo para corpo e rotores:

1. **Constante ou em degraus:** a entidade `Wind` do mundo (`<wind><linear_velocity>`), que o
   `MulticopterMotorModel` já lê. O novo plugin lê o mesmo componente (`components::WindLinearVelocity`)
   e fica coerente de graça.
2. **Turbulência:** Dryden (MIL-F-8785C / MIL-HDBK-1797), filtros de 1ª e 2ª ordem por eixo,
   barato e determinístico com semente. Rodar **um** gerador no plugin e escrever o resultado na
   entidade `Wind` faz os rotores do Gazebo verem a mesma rajada. É uma variável global, o que é
   aceitável porque as distâncias entre drones são pequenas frente à escala de turbulência
   (L ≈ 50–500 m). RotorPy faz a mesma hipótese.
3. **Esteira do alvo (opcional):** campo de downwash pré-tabelado (cone com v_i ≈ √(T/2ρA) que decai
   com a distância) somado a v_vento só no perseguidor quando estiver dentro do cone. Kiran et al.
   2025 e arXiv 2403.13321 dão a forma e a escala.
