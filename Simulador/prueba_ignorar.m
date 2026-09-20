clc; clear; close all;
%parametros
dt = 0.01;
t_final = 200;
time = 0:dt:t_final;
N = length(time);

V_a = 25;
wind_steady = [5;-2;0];
turbulance = 'light';
T_total = eye(3);

x_gust = zeros(5,1);

gust_b_log = zeros(3,N);
v_w_i_log = zeros(3,N);

for k = 1:N
    noise = randn(3,1)/sqrt(dt);
    % k1
    [~, dx1, ~] = dryden_wind(x_gust, V_a, T_total, wind_steady, turbulance, noise);
    
    % k2
    [~, dx2, ~] = dryden_wind(x_gust + 0.5*dt*dx1, V_a, T_total, wind_steady, turbulance, noise);
    
    % k3
    [~, dx3, ~] = dryden_wind(x_gust + 0.5*dt*dx2, V_a, T_total, wind_steady, turbulance, noise);
    
    % k4
    [~, dx4, ~] = dryden_wind(x_gust + dt*dx3, V_a, T_total, wind_steady, turbulance, noise);
    
    % Actualización del estado x_gust mediante RK4
    x_gust = x_gust + (dt/6) * (dx1 + 2*dx2 + 2*dx3 + dx4);
    
    % Evaluamos la salida con el estado ya actualizado
    [v_w_i, ~, gust_b] = dryden_wind(x_gust, V_a, T_total, wind_steady, turbulance, noise);
    
    % Guardamos los datos para graficar
    gust_b_log(:, k) = gust_b;
    v_w_i_log(:, k)  = v_w_i;
end

% --- 5. GRÁFICAS ---
figure('Name', 'Prueba del Modelo de Dryden con RK4', 'NumberTitle', 'off');

% Subplot 1: Ráfagas en Ejes Cuerpo
subplot(2,1,1);
plot(time, gust_b_log(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(time, gust_b_log(2,:), 'g', 'LineWidth', 1.2);
plot(time, gust_b_log(3,:), 'b', 'LineWidth', 1.2);
grid on;
title('Ráfagas de Turbulencia en Marco Cuerpo (w_{gust}^b)');
xlabel('Tiempo (s)'); ylabel('Velocidad (m/s)');
legend('u_g (Longitudinal)', 'v_g (Lateral)', 'w_g (Vertical)');

% Subplot 2: Viento Total en Marco Inercial
subplot(2,1,2);
plot(time, v_w_i_log(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(time, v_w_i_log(2,:), 'g', 'LineWidth', 1.2);
plot(time, v_w_i_log(3,:), 'b', 'LineWidth', 1.2);
grid on;
title('Viento Total en Marco Inercial (v_w^i = w_{steady} + w_{gust}^i)');
xlabel('Tiempo (s)'); ylabel('Velocidad (m/s)');
legend('V_{w,x}', 'V_{w,y}', 'V_{w,z}');

std(gust_b_log,0,2)
