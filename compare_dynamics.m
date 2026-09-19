%% compara_dinamicas.m
% Compara airplane_dynamics_rk4 (vieja) vs airplane_dynamics_opt (nueva)
% con inputs IDÉNTICOS, sin pasar por base_simulator ni CONDOR_M1.
% roll_rad se mantiene CONSTANTE durante todo el escenario (a propósito:
% así aislamos la física de vuelo, no la lógica de rolido/maniobra).

clc; clear; close all;

%% --- Config común ---
MTOW     = 8;
S_ref    = 1.386;
cd0      = 0.016;
rho      = 1.225;
dt       = 0.1;
t_max    = 30;        % [s] tiempo de cada escenario (deja asentar el equilibrio)
throttle = 1800;      % [us]
S_Banner = 0; cd_Banner = 0;
V_inicial = 20;        % [m/s]

prop_file  = 'PER3_20x10E.dat';
motor_file = 'Scorpion A-5025-310kv.dat';
polar_file = 'CONDOR.dat';

repo_root = fileparts(mfilename('fullpath'));
addpath(fullfile(repo_root, 'simulador'));
addpath(fullfile(repo_root, 'datos', 'helices'));
addpath(fullfile(repo_root, 'datos', 'motores'));
addpath(fullfile(repo_root, 'datos', 'polares'));
addpath(fullfile(repo_root, 'legacy'));   % para encontrar airplane_dynamics_rk4.m

PROP_TABLE  = prop(prop_file);
MOTOR_TABLE = leer_dat_mot(motor_file);
MOTOR_TABLE.torque = abs(MOTOR_TABLE.torque);
AVION_TABLE = leer_dat_avion(polar_file);

n_pasos = round(t_max/dt);

escenarios = struct('nombre', {'Recta (crucero)', 'Viraje 60 grados'}, 'roll_deg', {0, 60});

for e = 1:numel(escenarios)
    esc = escenarios(e);
    fprintf('\n========== %s ==========\n', esc.nombre);

    % ---------- VIEJO: airplane_dynamics_rk4 ----------
    clear airplane_dynamics_rk4   % resetea cualquier estado interno entre corridas
    x=0;y=0;z=0; v_x=V_inicial;v_y=0;v_z=0;
    roll_rad=deg2rad(esc.roll_deg); pitch_rad=0; yaw_rad=0;
    Energy_old=0; t=0; inform_old=[];
    for k = 1:n_pasos
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,inform_old,Energy_old,t] = ...
            airplane_dynamics_rk4(MTOW,dt,rho,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, inform_old, Energy_old, t, S_Banner, cd_Banner);
    end
    V_old = sqrt(v_x^2+v_y^2+v_z^2);
    CL_old = inform_old(11,end);
    Thrust_old = inform_old(14,end);
    [~, Motor_row] = motor(MOTOR_TABLE, throttle);  % corriente vieja: solo depende del throttle
    I_old = Motor_row.current;

    % ---------- NUEVO: airplane_dynamics_opt ----------
    clear airplane_dynamics_opt   % resetea persistent (omega_seed, C_caos) entre corridas
    x=0;y=0;z=0; v_x=V_inicial;v_y=0;v_z=0;
    roll_rad=deg2rad(esc.roll_deg); pitch_rad=0; yaw_rad=0;
    Energy_new=0; t=0;
    inform_new = zeros(27, n_pasos);
    for k = 1:n_pasos
        [x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy_new,t] = ...
            airplane_dynamics_opt(MTOW,dt,rho,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                PROP_TABLE, MOTOR_TABLE, AVION_TABLE, Energy_new, t, S_Banner, cd_Banner);
        inform_new(:,k) = log_step;
    end
    V_new = sqrt(v_x^2+v_y^2+v_z^2);
    CL_new = inform_new(11,end);
    Thrust_new = inform_new(14,end);
    I_new_final = inform_new(24,end);
    I_new_peak  = max(inform_new(24,:));

    % ---------- Reporte ----------
    fprintf('VIEJO -> V=%.2f m/s | CL=%.3f | Thrust=%.1f N | I=%.1f A (cte, no depende del vuelo) | Energy=%.4f Ah\n', ...
        V_old, CL_old, Thrust_old, I_old, Energy_old);
    fprintf('NUEVO -> V=%.2f m/s | CL=%.3f | Thrust=%.1f N | I_final=%.1f A | I_pico=%.1f A | Energy=%.4f Ah\n', ...
        V_new, CL_new, Thrust_new, I_new_final, I_new_peak, Energy_new);
    fprintf('Delta V: %+.1f %% | Delta CL: %+.1f %% | Delta Thrust: %+.1f %% | Delta Energy: %+.1f %%\n', ...
        100*(V_new-V_old)/V_old, 100*(CL_new-CL_old)/CL_old, ...
        100*(Thrust_new-Thrust_old)/Thrust_old, 100*(Energy_new-Energy_old)/max(Energy_old,1e-9));
end