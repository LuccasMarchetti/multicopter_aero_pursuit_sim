function P = parametros()
%PARAMETROS Single source of the study parameters (x500, PX4, delays, yaml gains).
%   P = parametros() returns a struct used by all scripts.
%   Keep in sync with ros2_ws/src/aero_pursuit/config/aero_pursuit.yaml.

%% Tempo
P.Ts    = 0.05;     % pursuit_controller_node period [s]
P.h     = 0.002;    % "plant" integration step in the simulation [s]
P.fcam  = 30;       % camera / tracker rate [Hz]
P.tau   = 0.10;     % measurement delay (camera + VitTrack + DDS) [s]
P.ruido = 0.10;     % position measurement standard deviation [m]

%% x500 (vehicle_params.hpp)
P.m   = 2.0 + 4*0.016076923076923075;   % [kg]
P.g   = 9.8;
P.Ixx = 0.02166666666666667;  P.Iyy = P.Ixx;  P.Izz = 0.04;
P.kf  = 8.54858e-06;  P.wmax = 1000;  P.wmin = 150;
P.Tmax = 4*P.kf*P.wmax^2;                       % [N]
P.D   = 4*8.06428e-05*sqrt(P.m*P.g/4/P.kf);     % arrasto linear [N/(m/s)]
P.tau_max = 4*0.174*min(P.kf*P.wmax^2 - P.m*P.g/4, P.m*P.g/4);  % torque rol/arf [N m]

%% PX4 (firmware defaults; check with "param show")
P.px4.Kp_v = 1.8;   % MPC_XY_VEL_P_ACC
P.px4.Ki_v = 0.4;   % MPC_XY_VEL_I_ACC
P.px4.Kd_v = 0.2;   % MPC_XY_VEL_D_ACC (on the measured velocity)
P.px4.wa   = 4.0;   % MC_PITCH_P -> attitude loop bandwidth [rad/s]
P.px4.tilt_max = deg2rad(45);  % MPC_TILTMAX_AIR

%% Mission / limits (aero_pursuit.yaml)
P.d     = 5.0;      % target_distance [m]
P.vmax  = 2.0;      % max_vel
P.amax  = 3.0;      % max_acc

%% Estimador (target_estimator_node)
P.kf_sigma_a = 0.5;     % process_noise_std
P.kf_sigma_z = 0.10;    % measurement_noise_std
P.kf_sigma_v0 = 1.0;    % initial_velocity_std
P.kf_gate = 16.27;      % gating_chi2
P.kf_max_rej = 5;       % max_gate_rejections
P.kf_compensar = true;  % compensate_chaser_motion

%% Gains of each controller (yaml)
P.pid = struct('kp',0.8,'ki',0.05,'kd',0.2,'ff',1.0,'ilim',2.0,'izona',1.0);
P.lqr = struct('qp',10,'qv',2,'r',0.5);
P.lqi = struct('qp',10,'qv',2,'qi',2,'r',0.5,'ilim',5.0,'izona',2.0);
P.smc = struct('lambda',0.8,'ks',1.0,'k0',0.5,'kmin',0,'kmax',3.0, ...
               'gamma',0.5,'sigma',0.1,'phi',0.3);
P.mpc = struct('N',10,'qp',10,'qv',2,'r',0.5,'growth',1.15,'wN',2.0, ...
               'iter',50,'rho',200,'outer',4,'predict_los',true,'vel_con',true);
P.her = struct('wT',1.0,'wj',0.02,'wlim',50,'Tmin',0.4,'Tmax',8.0,'M',12);
P.pn  = struct('N',4.0,'K',1.0,'alpha',0.5,'klat',1.0,'kr',0.8,'krd',1.2,'vmin',1.0);
end
