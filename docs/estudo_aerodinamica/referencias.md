# Referências

## Código e documentação lidos

- gz-sim 8, `MulticopterMotorModel.cc`:
  https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc
- gz-sim `AdvancedLiftDrag`: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AdvancedLiftDrag.html
- gz-sim `WindEffects`: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WindEffects.html
  e PR #1357: https://github.com/gazebosim/gz-sim/pull/1357
- gz-sim `Hydrodynamics`: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Hydrodynamics.html
- PX4 local: `Tools/simulation/gz/{server.config,worlds/lawn.sdf,models/x500*}`,
  `src/modules/simulation/simulator_sih/{sih.cpp,aero.hpp}`, `src/modules/ekf2/params_drag.yaml`
- Pegasus Simulator: https://pegasussimulator.github.io/PegasusSimulator/source/features/vehicles.html;
  código `logic/dynamics/linear_drag.py`, `logic/thrusters/quadratic_thrust_curve.py`,
  `logic/vehicles/multirotor.py` em https://github.com/PegasusSimulator/PegasusSimulator
- AirSim `FastPhysicsEngine.hpp`: https://github.com/microsoft/AirSim
- RotorPy `vehicles/multirotor.py`, `hummingbird_params.py`: https://github.com/spencerfolk/rotorpy
- ArduPilot SITL `libraries/SITL/SIM_Frame.cpp`: https://github.com/ArduPilot/ardupilot
- MuJoCo, fluid forces: https://mujoco.readthedocs.io/en/stable/computation/fluid.html

## Artigos e relatórios

- Russell, C.; Jung, J.; Willink, G.; Glasner, B. *Wind Tunnel and Hover Performance Test Results for
  Multicopter UAS Vehicles.* AHS Forum 72, 2016; NASA/TM-2018-219758.
  https://rotorcraft.arc.nasa.gov/Publications/files/72-2016-374.pdf (Figs. 17–30 usadas aqui)
- Foster, J. V.; Hartman, D. *High-Fidelity Multi-Rotor Unmanned Aircraft System Simulation
  Development for Trajectory Prediction Under Off-Nominal Flight Dynamics.* AIAA Aviation 2017.
  https://ntrs.nasa.gov/api/citations/20170006073/downloads/20170006073.pdf. Software NASA LAR-19479-1.
- Faessler, M.; Franchi, A.; Scaramuzza, D. *Differential Flatness of Quadrotor Dynamics Subject to
  Rotor Drag for Accurate Tracking of High-Speed Trajectories.* IEEE RA-L, 2018. arXiv:1712.02402
- Bauersfeld, L.; Kaufmann, E.; Foehn, P.; Sun, S.; Scaramuzza, D. *NeuroBEM: Hybrid Aerodynamic
  Quadrotor Model.* RSS 2021. arXiv:2106.08015
- Folk, S.; Paulos, J.; Kumar, V. *RotorPy: A Python-based Multirotor Simulator with Aerodynamics for
  Education and Research.* 2023. arXiv:2306.04485
- Hattenberger, G.; Bronz, M.; Condomines, J.-P. *Evaluation of drag coefficient for a quadrotor
  model.* Int. J. Micro Air Vehicles 15, 2023 (IMAV 2022). https://www.imavs.org/papers/2022/4.pdf
- Sun, S.; de Visser, C. C.; Chu, Q. P. *Quadrotor Gray-Box Model Identification from High-Speed Flight
  Data.* J. Aircraft, 2019; e *Aerodynamic Model Identification of a Quadrotor Subjected to Rotor
  Failures in the High-Speed Flight Regime.* RA-L, 2019 (TU Delft)
- Gill, R.; D'Andrea, R. *Propeller Thrust and Drag in Forward Flight.* IEEE CCTA 2017; e
  *Computationally Efficient Force and Moment Models for Propellers in UAV Forward Flight
  Applications.* Drones 3(4):77, 2019
- Song, Y. et al. *Flightmare: A Flexible Quadrotor Simulator.* CoRL 2020. arXiv:2009.00563
- Panerati, J. et al. *Learning to Fly: a Gym Environment with PyBullet Physics for Reinforcement
  Learning of Multi-agent Quadcopter Control.* IROS 2021. arXiv:2103.02142 (efeito solo de Shi et
  al., *Neural Lander*, ICRA 2019)
- Jacinto, M.; Pinto, J.; Patrikar, J.; Keller, J.; Cunha, R.; Scherer, S.; Pascoal, A. *Pegasus
  Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation.* ICUAS 2024
- Talaeizadeh, A. et al. *Optimal-time Quadcopter Descent Trajectories Avoiding the Vortex Ring and
  Autorotation States.* arXiv:1909.09069; Mechatronics 68, 2020; RA-L 2021
- Kiran, A. et al. *Influence of Static and Dynamic Downwash Interactions on Multi-Quadrotor Systems.*
  RSS 2025. arXiv:2507.09463
- *Downwash-aware control from drone flight data* (16 h de voo, vários tamanhos). arXiv:2403.13321
- Mahony, R.; Kumar, V.; Corke, P. *Multirotor Aerial Vehicles: Modeling, Estimation, and Control of
  Quadrotor.* IEEE RAM, 2012 (rotor drag e blade flapping lineares)
- Huang, H.; Hoffmann, G.; Waslander, S.; Tomlin, C. *Aerodynamics and Control of Autonomous Quadrotor
  Helicopters in Aggressive Maneuvering.* ICRA 2009 (empuxo × ângulo de ataque, flapping)
- Cheeseman, I.; Bennett, W. *The Effect of the Ground on a Helicopter Rotor in Forward Flight.*
  ARC R&M 3021, 1955
- Hoerner, S. F. *Fluid-Dynamic Drag.* 1965 (C_D de placa plana e corpos rombudos)
- MIL-HDBK-1797 / MIL-F-8785C (modelo de turbulência Dryden)

Ferramentas citadas para gerar tabelas: OpenFOAM, SU2, OpenVSP/VSPAERO, DUST
(https://www.dust-project.org), trimesh (Python).
