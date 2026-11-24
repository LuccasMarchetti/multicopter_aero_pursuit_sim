# 5. Viabilidade técnica e custo computacional

## 5.1 Onde o código entraria (sem recompilar o Gazebo nem mexer no PX4)

O gz-sim carrega *systems* como bibliotecas dinâmicas (`gz::sim::System` +
`ISystemConfigure`/`ISystemPreUpdate`). Um plugin compilado no próprio workspace e achado por
`GZ_SIM_SYSTEM_PLUGIN_PATH` atende a regra do projeto: plugins de runtime no nosso pacote são
permitidos, recompilar o Gazebo não.

Duas formas de ligar o plugin:

| Opção | Como | Prós | Contras |
|---|---|---|---|
| **(1) System de mundo** (recomendado) | `<plugin>` no mundo, ou `entity_type="world"` num `server.config` do projeto, com a lista `<vehicle model="x500_depth_0" link="base_link" table="x500.yaml"/>`. Aplica a modelos que aparecem depois (o PX4 spawna o x500 depois do mundo) | Não toca nos SDFs do PX4. Um plugin serve todos os drones e um único gerador de vento atende todos | Precisa de um mundo ou `server.config` próprio (hoje o `lawn.sdf` é do PX4. Copiar para o pacote ou usar um `server.config` local) |
| (2) Plugin de modelo | `<plugin>` dentro do `model.sdf` | Padrão do Gazebo | Exige copiar ou sobrepor `x500_depth` (hoje vem do PX4). Os alvos em `aero_pursuit/gz_models/` são fáceis |

A física em cada `PreUpdate` (passo de 4 ms):

1. ler `WorldPose`, `WorldLinearVelocity` e `WorldAngularVelocity` do link (ativar com
   `Link::EnableVelocityChecks`);
2. ler o vento da entidade `Wind` (mais turbulência própria, se ativa);
3. calcular **v_ar**, α, β e V, consultar a tabela (seção 4.4) e montar F e M;
4. `Link::AddWorldWrench(ecm, F_mundo, M_mundo)`. Esse wrench **soma** com o que o
   `MulticopterMotorModel` já escreve no mesmo passo, e o sistema `Physics` consome e zera ao fim.

Há um detalhe de estabilidade numérica. Arrasto quadrático com integração explícita a 250 Hz é
estável com folga. O tempo característico é m/(ρ CdA V) ≈ 2,064/(1,225·0,03·15) ≈ 3,7 s, muito
maior que 4 ms. Amortecimento de rotação com I ≈ 0,02 kg·m² também é tranquilo para coeficientes
realistas. Mesmo assim, convém limitar |M_amort| ≤ I|ω|/dt.

## 5.2 Custo medido

`scripts/bench_lookup.cpp` (g++ -O2, nesta máquina): transformação, α/β (`atan2`, `asin`),
interpolação trilinear de 6 coeficientes em uma tabela 73×37×12 com acessos aleatórios à memória
(pior caso de cache) e montagem do wrench:

```
custo médio por avaliação: ~150 ns
a 250 Hz × 10 veículos: ~0,04 % de um núcleo
```

Para comparação, o `DynamicMpcController` gasta ~140 µs por ciclo, ~1000× mais. A renderização das
câmeras (RGB + profundidade) domina o RTF de longe. **O custo do modelo tabelado é desprezível.**
Mesmo um BEMT por rotor em tempo real (~10 elementos × 2 pás × 4 rotores, com iteração de inflow)
ficaria na casa de µs por passo. O motivo para tabelar é robustez e determinismo, não CPU.

O que pode pesar de fato:

- **gz-transport e ROS:** publicar telemetria do plugin (forças, α, β) a 250 Hz por veículo. A saída
  é limitar a 20–50 Hz, só para depuração ou o painel da GCS, no mesmo espírito do
  `gazebo_tf_bridge_node` (`min_period_ns_`).
- **Campo de vento espacial** (CFD ou LES pré-calculado, tabela 4D): memória, não CPU. Fora do
  escopo.

## 5.3 Opção sem código para um experimento rápido: `Hydrodynamics`

O sistema `gz::sim::systems::Hydrodynamics` (do próprio gz-sim) calcula amortecimento 6-DOF linear
e **quadrático \|v\|v**, com termos cruzados, sobre a velocidade relativa a uma "corrente". Com
massa adicional e Coriolis desligados (`<disable_added_mass>`, `<disable_coriolis>`) ele vira um
arrasto de corpo quadrático com vento:

```xml
<plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
  <link_name>base_link</link_name>
  <xUabsU>-0.0184</xUabsU>   <!-- −½ ρ CdA_x, CdA_x = 0,030 m² -->
  <yVabsV>-0.0184</yVabsV>
  <zWabsW>-0.0600</zWabsW>   <!-- topo, ~3× -->
  <kPabsP>-0.002</kPabsP> <mQabsQ>-0.002</mQabsQ> <nRabsR>-0.001</nRabsR>
  <disable_added_mass>true</disable_added_mass>
  <disable_coriolis>true</disable_coriolis>
  <default_current>0 0 0</default_current>  <!-- = vento, mundo -->
</plugin>
```

Serve para medir em 1 hora o efeito do arrasto parasita no comportamento do PX4 e do controlador
de perseguição (velocidade máxima, overshoot, tilt) antes de investir no plugin próprio.
Limitações: diagonal por eixo (sem forma em α e β), vento desacoplado da entidade `Wind` (os rotores
não veem a mesma corrente), nomenclatura e sinais de Fossen (NED/FRD, conferir sinais), e o plugin
pensado para água. Exige colocar o `<plugin>` no modelo, ou seja, uma cópia do `x500_depth` no
pacote.

## 5.4 Impactos no restante do projeto

| Componente | Efeito | Ação |
|---|---|---|
| `DynamicMpcController` / `VehiclePhysics` (D = 0,248 N·s/m linear) | Modelo de predição fica otimista em velocidade | Linearizar D_eff(v₀) = D_rotor + ρ CdA v₀ em torno da velocidade de referência, ou passar D como parâmetro na aba Controle da GCS |
| `control/flatness.hpp` (θ_ref = D v/(m g)) | Inclinação de cruzeiro subestimada | Mesmo D_eff, ou termo quadrático no feedforward (Faessler 2018 mostra que a planeza vale com arrasto linear. O quadrático entra como perturbação conhecida) |
| PX4 EKF2 (`EKF2_DRAG_CTRL`, `EKF2_BCOEF_X/Y`, `EKF2_MCOEF`) | Com arrasto realista, dá para **testar a estimação de vento do PX4** em SITL | BCOEF = m/CdA ≈ 2,064/0,03 ≈ 69 kg/m²; MCOEF ≈ 0,12 s⁻¹ (já coerente com o rotor drag do SDF) |
| Alvos (X3/X4/Tiltrotor via `MulticopterControl`) | Mesmo arrasto torna as trajetórias do alvo fisicamente consistentes | Tabela própria por modelo. O controlador do alvo compensa sozinho |
| Visão / profundidade | Mais tilt em alta velocidade muda o campo de visão da câmera fixa | Efeito real e desejável no estudo |
| Tempo real (RTF) | Desprezível (5.2) | — |
| GCS / Foxglove | Útil ver F, M, α, β e vento | Tópico de telemetria limitado em taxa. TF não muda |

## 5.5 Veredito

**Faz sentido implementar**, com escopo controlado:

- Fase 1 (corpo tabelado + vento coerente) tem custo de CPU ~0, não toca no Gazebo nem no PX4, e
  corrige a maior lacuna física identificada (velocidades de regime 2–3× maiores que o real acima de
  20° de tilt).
- O ganho **nos cenários atuais (2–5 m/s, sem vento) é pequeno**: arrasto do corpo de 0,07 a 0,5 N
  frente a 0,5–1,2 N de rotor drag. O valor aparece quando houver vento, rajadas, alvo rápido
  (≥ 8 m/s) ou manobras de frenagem forte. Esses são exatamente os casos que um TCC de perseguição
  quer mostrar como "robustos".
- Fases de rotor (inflow, VRS, efeito solo) e esteira só valem se os modos verticais ou a
  perseguição por baixo do alvo virarem foco.
- Alta fidelidade (CFD completo, NeuroBEM) não se justifica sem dados reais do x500 para validar.
