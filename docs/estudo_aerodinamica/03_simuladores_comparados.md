# 3. O que outros simuladores implementam

Levantamento pela fonte (código lido) quando possível, senão pela documentação ou pelo artigo.
"Código" indica que a fórmula foi conferida no repositório.

| Simulador | Rotor (empuxo / reação) | Rotor drag e flapping | Corpo | Vento | Custo / observação |
|---|---|---|---|---|---|
| **Gazebo Harmonic `MulticopterMotorModel`** (código) | k_f ω², k_m T, motor 1ª ordem | −\|ω\| c_d v⊥ e −\|ω\| c_r v⊥ | **nada** | subtrai `Wind` do mundo no rotor drag | Base atual do projeto |
| **Gazebo `LiftDrag` / `AdvancedLiftDrag`** (docs) | — | — | Asa: C_L(α) linear com estol. O *Advanced* usa derivadas de estabilidade (C_Lα, C_Yβ, C_mq …), placa plana pós-estol (`CD_fp_k1/k2`), `<cp>` | opcional | Feito para asas (FW/VTOL). Gerado pelo `avl_automation` do PX4 (AVL). Ruim para bluff body |
| **Gazebo `WindEffects`** (docs, PR #1357) | — | — | Força ∝ (v_vento − v_link) nos links com `<enable_wind>`, aplicada na origem do link | campo escalar por região, ruído | Aproximação grosseira; o próprio PR admite que sem ponto de aplicação faltam torques |
| **Gazebo `Hydrodynamics`** (docs) | — | — | Fossen 6-DOF: amortecimento linear e **quadrático \|v\|v** com termos cruzados, massa adicional, Coriolis | `default_current` ou tópico `/model/<ns>/ocean_current` | Feito para água, mas o amortecimento quadrático em relação a uma "corrente" é exatamente um arrasto de corpo 6-DOF. **Dá para testar sem escrever código** (ver 05) |
| **Pegasus Simulator** (Isaac Sim, código) | k ω² por rotor (mesmos 8,54858e-6 do Gazebo), reação k_r ω² **sem dinâmica de motor** | — | `LinearDrag([0.50, 0.30, 0.0])`: F = −diag(d) v_corpo, linear | **sem vento** no arrasto | Mais simples que o Gazebo atual. Ganho do Pegasus é a renderização e o PhysX em GPU, não a aerodinâmica |
| **AirSim** (código, `FastPhysicsEngine`) | C_T ρ n² D⁴ constante por hélice | — | **"drag faces"**: por face de caixa, F = −k ρ v_n² **n**, com v da face incluindo ω × r e torque r × F | subtrai vento | Placa plana por face: barata e já dá dependência com α, β e com rotação |
| **RotorPy** (UPenn, código) | k_η ω², k_m ω², motor 1ª ordem | H = −ω diag(k_d, k_d, k_z) v_hub (v_hub inclui ω × r), flapping M = −k_flap ω (v_hub × e_z), sust. translacional k_h v_xy² | parasita −\|v\| diag(c_Dx, c_Dy, c_Dz) v (quadrático) | campos espaço-temporais, Dryden | **A melhor referência de "modelo leve e completo"**. Parâmetros do Hummingbird publicados |
| **ArduPilot SITL** (código, `SIM_Frame.cpp`) | modelo de motor/ESC/bateria | momentum drag ∝ v √(T ρ A_disco) | bluff body por eixo: ½ρ CdA v² | sim, via v_ar | CdA calibrado por **um ponto** (V_ref, θ_ref). Imprime os `EK3_DRAG_BCOEF/MCOEF` coerentes |
| **PX4 SIH** (código, `sih.cpp`) | k ω² | — | quad: −K_DV v_ap e −K_DW ω (1ª ordem); asas por `aero.hpp` (placa plana no pós-estol) | `SIH_WIND_N/E` | Roda na própria Pixhawk; fidelidade baixa |
| **Flightmare / Agilicious** (artigo) | k ω² | matriz diagonal constante (linear) | — | — | Foco em renderização e RL |
| **gym-pybullet-drones** (artigo) | k ω² | arrasto ∝ ω v (Forster) | — | — | Modos PyBullet + drag / efeito solo (Shi et al.) / downwash (Zhou, DSL) |
| **MuJoCo** (docs) | via atuadores | — | modelo de fluido por elipsoide: arrasto rombudo + viscoso, sustentação de **Kutta** e Magnus, **massa adicional** | `wind` global | Genérico e barato. Mostra que o modelo "elipsoide equivalente" é uma alternativa à tabela |
| **NASA LaRC (Foster & Hartman)** (artigo) | base de dados de túnel em atitude total | — | base de dados de túnel estático/dinâmico, 6-DOF, **toda a envoltória** | — | Matlab/Simulink, uso governamental. É a referência metodológica para tabelas |
| **NeuroBEM** (artigo) | BEM por pá e por elemento | BEM | rede neural residual | — | Alta fidelidade e custo maior. Precisa de dados de voo em mocap |

## Leitura

- **Nenhum simulador "de robótica" popular tem um modelo de corpo tabelado em (α, β).** O que existe
  é constante por eixo (RotorPy, ArduPilot, EKF2) ou placa plana por face (AirSim). Tabelas são a
  prática da indústria aeronáutica (JSBSim, bases da NASA) e de quem fez túnel (TU Delft, NASA).
  Para um TCC isso é um diferencial técnico real e não exagerado.
- O Gazebo já tem a metade "rotor" num nível comparável a Flightmare e Pegasus. A lacuna está no
  corpo e no vento.
- O Pegasus **não** é uma referência de fidelidade aerodinâmica: arrasto linear constante, sem vento,
  sem dinâmica de motor. Vale citá-lo como contraexemplo.
- RotorPy e ArduPilot mostram o modelo "leve e suficiente" que a literatura aceita. A tabela (α, β)
  é a extensão natural quando se quer momentos e anisotropia.
