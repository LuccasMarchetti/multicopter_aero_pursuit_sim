# MPC próprio: onde escrever, o que ler e como recompilar

Branch `feat/mpc-from-scratch`. Todas as leis de controle anteriores, o lag, o gimbal, o fallback de ROI,
a banda morta e as políticas de yaw (`face_target`...) e vertical (`los`, `match_altitude`...) foram
removidos. Sobrou **uma lei**, `control::Mpc`: MPC translacional de 6 estados resolvido pelo acados.
Câmera fixa (`x500_depth`).

## 1. Onde escrever

| Arquivo | O que é |
|---|---|
| `ros2_ws/src/aero_pursuit/acados/pursuit_ocp.py` | **Modelagem**: estados, entradas, dinâmica discreta, custo, restrições, opções do solver, N e dt. |
| `ros2_ws/src/aero_pursuit/acados/generate.py` | Gera o código C do solver + `pursuit_mpc_dims.hpp` (N, DT, tamanhos, pesos padrão). Chamado pelo CMake. |
| `ros2_ws/src/aero_pursuit/src/control/mpc.cpp` | **Uso do solver a cada ciclo**: x0, referência, pesos/limites da GCS, solve, comando. |
| `ros2_ws/src/aero_pursuit/include/aero_pursuit/control/mpc.hpp` | Interface da classe `Mpc`. |
| `ros2_ws/src/aero_pursuit/include/aero_pursuit/control/pursuit_controller.hpp` | **Contrato**: `PursuitState` (entrada), `ControlCommand` (saída). |
| `ros2_ws/src/aero_pursuit/test/test_mpc.cpp` | Testes em malha fechada com o próprio modelo. |

## 2. Modelo atual

```
x = [p_rel; v_rel] (6)      p_rel = p_alvo - p_perseguidor, v_rel idem (ENU, centrado no perseguidor)
u = a_perseguidor (3)
x[k+1] = A x[k] - B u[k],   A = [[I, dt I], [0, I]],  B = [[dt²/2 I], [dt I]]     (alvo a velocidade constante)
custo  = Σ_k ||[x_k; u_k] - [p_ref; 0; 0]||²_W  +  ||x_N - [p_ref; 0]||²_We
W = diag(q_p, q_v, r_a),  We = q_f · diag(q_p, q_v),  p_ref = standoff · p_rel/|p_rel| (fixo no horizonte)
-a_max <= u <= a_max por eixo (a_max_xy em x e y, a_max_z em z)
```

N = 20, dt = 0,1 s (horizonte de 2 s). Solver: SQP-RTI + HPIPM (o problema é um QP: uma iteração é exata).
Sem yaw: `cmd.yaw` vazio, o nó mantém o rumo medido no engajamento.

## 3. Números da GCS

Aba **Controle** → "MPC (seu controlador)": lista nome + valor = parâmetro ROS `mpc.params.<nome>` (double) →
`Mpc::setParameter`. Nomes que o MPC lê: `q_px q_py q_pz q_vx q_vy q_vz r_ax r_ay r_az q_f a_max_xy a_max_z`.
Valores iniciais em `config/aero_pursuit.yaml` (`mpc.params`); remover um nome na GCS volta ao padrão do
`pursuit_ocp.py`. `target_distance` (standoff) e `max_vel` (saturação de segurança de |v| no nó) ficam em
"Saída e limites".

## 4. Conexões (nó `PursuitControllerNode`, `src/nodes/pursuit_controller_node.cpp`)

| Função | Linha | O que faz |
|---|---|---|
| `onTarget` | 142 | `/target_3d_state_filtered` → `target_` (p_rel, v_rel, carimbo) |
| `onBearing` | 152 | `/target_bearing_enu` → `bearing_` (caixa/linha de visada) |
| `onLocalPosition` | 174 | velocidade medida e rumo da PX4 |
| `onOdometry` | 184 | atitude/taxas/velocidade; saídas diretas |
| `controlLoop` | 228 | timer de 20 Hz: supervisor (TRACKING/COASTING/APPROACH/LOST), trava o rumo no engajamento |
| `runController` | 301 | monta o `PursuitState`, chama `Mpc::compute`, `limitVelocity`, escolhe a saída |
| `publishSetpoint` | 476 | `trajectory` → `/offboard/trajectory_setpoint` (velocidade + aceleração feedforward + yaw) |

`OffboardControlNode` republica `/offboard/*` em `/fmu/in/*`, arma/decola e segura posição sem setpoint.

## 5. Recompilar e rodar

Editou `pursuit_ocp.py`? O CMake detecta (hash dos `.py`) e regenera o solver na próxima compilação.
Editou só `mpc.cpp`? Compilação C++ normal.

```bash
aero build aero_pursuit          # nativo: colcon build; docker: no volume do contêiner
aero sim start                   # ou Simulação > Iniciar na GCS
```

Testes: `colcon test --packages-select aero_pursuit && colcon test-result --verbose`
(ou `./ros2_ws/build/aero_pursuit/test_control --gtest_filter='Mpc*'`).

acados: `ros2_ws/install_acados.sh` (v0.6.0 em `~/.local/share/acados`; `install.sh` e as imagens Docker o chamam).
Imagens que precisam ser reconstruídas depois de mudar dependências: `aero image` (GCS/Gazebo em Docker),
`isaac/host.sh build` (perseguidor do Isaac), `docker/pursuer.sh build <usuario@ip>` (Raspberry Pi).

## 6. O que ficou de fora

- APPROACH (sem profundidade): `Mpc::compute` devolve velocidade zero (paira); a ação com só a caixa é sua.
- Aceleração do alvo (`a_target`, Singer) e limites de velocidade não entram no modelo ainda.
- `ros2_ws/{run,tune}_study.py` e os guias (`docs/`, `guia_controle/`, `latex/`) ainda descrevem as leis antigas.
