clc
clear
close all
disp('M1');
% =========================================================================
% --- CONFIGURACIÓN DE LA SIMULACIÓN ---
% =========================================================================
MTOW=8;      %kg Por dios tocar
ro=1.255;
S_ref=1.386;
S=8;
cd0=0.05;%Factor de Corrección
t_n=0.1;
% --- Banner
l_Banner=0;
S_Banner=0;
cd_Banner=0;

repo_root = fileparts(fileparts(mfilename('fullpath')));  % sube un nivel desde Legacy/ a la raíz del repo
addpath(fullfile(repo_root, 'simulador'));
addpath(fullfile(repo_root, 'datos', 'helices'));
addpath(fullfile(repo_root, 'datos', 'motores'));
addpath(fullfile(repo_root, 'datos', 'polares'));

throttle=((1500+(500/1024)*766)+80);
prop1="PER3_20x10E.dat";
v_x=118/3.6;

%123kph 28.7A
% --- Archivos de Datos ---

motor1='Scorpion A-5025-310kv.dat';
plane1='CONDOR.dat';

% --- Plan de Vuelo ---
roll_1 = -60; 
roll_2 = -60;
roll_3 = -60;



% =========================================================================
% --- OPTIMIZACIÓN: Cargar archivos UNA SOLA VEZ ---
% =========================================================================
disp('Cargando archivos de datos (una sola vez)...');

PROP_TABLE = prop(prop1);

MOTOR_TABLE = motor(motor1);
MOTOR_TABLE.torque = abs(MOTOR_TABLE.torque);
AVION_TABLE = avion_cl(plane1);
disp('Archivos cargados en memoria.');

% =========================================================================
%% --- CONDICIONES INICIALES (t=0) ---
% =========================================================================
t=0;
x=0; y=0; z=0;
 v_y=0; v_z=0;
Energy=0;
pitch_rad=deg2rad(0);
roll_rad=deg2rad(0);
yaw_rad=deg2rad(0);

max_pasos = 5000;              % estimación generosa (a t_n=0.1s, cubre 500s de vuelo)
inform_mat = zeros(29, max_pasos);
step_idx = 0;

%% Despegue (Sección comentada)
while pitch_rad>-0.5
    break
    % ... (código de despegue omitido) ...
end
pitch_rad=0;

% =========================================================================
%% --- BUCLE DE SIMULACIÓN ---
% =========================================================================

%% Pierna #1
x0=x; y0=y;
disp("Pierna   #1 ñeri")
while sqrt((x-x0)^2+(y-y0)^2)<100
    pitch_rad=0;
    % --- LLAMADA OPTIMIZADA ---
    % Pasamos las TABLAS (ej: PROP_TABLE) en lugar de los NOMBRES (ej: prop1)
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
pitch_rad=0;

%% AUTO Giro #1
roll_rad_obj=deg2rad(roll_1);
t_obj=1;
roll_dif=-roll_rad_obj+roll_rad;
disp("AutoGyro 1a ñeri")
while  abs(roll_rad - roll_rad_obj) > deg2rad(2)
    pitch_rad=0;
    roll_rad=roll_rad+t_n*(-roll_dif)/t_obj;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
disp("AutoGyro 1b ñeri")
while abs(yaw_rad - deg2rad(180-13)) > deg2rad(2)
    pitch_rad=0;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
roll_rad_obj=deg2rad(0);
t_obj=1;
roll_dif=roll_rad-roll_rad_obj;
disp("AutoGyro 1c ñeri")
while  abs(roll_rad - roll_rad_obj) > deg2rad(2)
    pitch_rad=0;
    roll_rad=roll_rad+t_n*(-roll_dif)/t_obj;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
roll_rad=deg2rad(0);
%yaw_rad=deg2rad(180);

%% Pierna #2
x0=x; y0=y;
disp("Pierna   #2 ñeri")
while sqrt((x-x0)^2+(y-y0)^2)<130
    pitch_rad=0;
    roll_rad=deg2rad(0);
    %yaw_rad=deg2rad(180);
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end

%% AutoGyro #2
roll_rad_obj=deg2rad(roll_2);
t_obj=1;
roll_dif=-roll_rad_obj+roll_rad;
disp("AutoGyro 2a ñeri")
while  abs(roll_rad - roll_rad_obj) > deg2rad(2)
    pitch_rad=0;
    roll_rad=roll_rad-t_n*(roll_dif)/t_obj;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
disp("AutoGyro 2b ñeri")
% OJO: El objetivo de yaw aquí (-500) es lo mismo que (-140)
% roll_2 es -73, girará a la izquierda (negativo)
while abs(yaw_rad - deg2rad(360+180-13)) > deg2rad(2) % Corregido de -520 a -140
    %rad2deg((yaw_rad - deg2rad(360)))
    
    pitch_rad=0;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
roll_rad_obj=deg2rad(0);
t_obj=1;
roll_dif=-roll_rad_obj+roll_rad;
disp("AutoGyro 2c ñeri")
while  abs(roll_rad - roll_rad_obj) > deg2rad(2)
    pitch_rad=0;
    roll_rad=roll_rad+t_n*(-roll_dif)/t_obj;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end

%% Pierna #3
x0=x; y0=y;
roll_rad=0;
disp("Pierna   #3 ñeri")
while sqrt((x-x0)^2+(y-y0)^2)<90
    pitch_rad=0;
    roll_rad=0;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end

%% AUTO Giro #3
roll_rad_obj=deg2rad(roll_3);
t_obj=1;
roll_dif=-roll_rad_obj+roll_rad;
disp("AutoGyro 3a ñeri")
while  abs(roll_rad - roll_rad_obj) > deg2rad(2)
    pitch_rad=0;
    roll_rad=roll_rad+t_n*(-roll_dif)/t_obj;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
disp("AutoGyro 3b ñeri")
while abs(yaw_rad - deg2rad(-13+720)) > deg2rad(5)

    pitch_rad=0;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end
roll_rad_obj=deg2rad(0);
t_obj=1;
roll_dif=roll_rad-roll_rad_obj;
disp("AutoGyro 3c ñeri")
while  abs(roll_rad - roll_rad_obj) > deg2rad(2)
    pitch_rad=0;

    roll_rad=roll_rad+t_n*(-roll_dif)/t_obj;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end

%% Pierna #4
x0=0; y0=y;
disp("Pierna    #4 ñeri")
while sqrt((x-x0)^2+0*(y-y0)^2)>5
    pitch_rad=0;
    roll_rad=0;
[x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,log_step,Energy,t] = airplane_dynamics_opt(MTOW,t_n,ro,S_ref,x,y,z,v_x,v_y,v_z,roll_rad,pitch_rad,yaw_rad,throttle,cd0, ...
                                                                                          PROP_TABLE, MOTOR_TABLE, AVION_TABLE, ...
                                                                                          Energy,t,S_Banner,cd_Banner);
step_idx = step_idx + 1; inform_mat(:, step_idx) = log_step;
end

disp("Listo")
% =========================================================================
%% --- GRÁFICOS Y RESULTADOS ---
% =========================================================================
% --- Llamada a la nueva función de ploteo ---
% Pasamos todos los parámetros necesarios para los gráficos y el resumen
%plot_results(inform, t, Energy, S, plane1, motor1, prop1, MTOW, throttle, t_n);
inform = inform_mat(:, 1:step_idx);
plot_results_report(inform, t, Energy, S, plane1, motor1, prop1, MTOW, throttle, t_n);


% --- EXPORTAR DATOS ---
tabla_vuelo = array2table(inform', 'VariableNames', {'x','y','z','v_x','v_y','v_z','a_x','a_y','a_z','t','cl','cd_total','drag','Thrust_N','LIFT_x','LIFT_y','LIFT_z','THRUST_x','THRUST_y','THRUST_z','DRAG_x','DRAG_y','DRAG_z','Corriente_real','E_wind_x','E_wind_y','roll_rad','omega'});
writetable(tabla_vuelo, 'datos_vuelo_avion.csv');
disp('✈️ Datos de vuelo');