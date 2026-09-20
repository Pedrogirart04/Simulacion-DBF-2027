function [v_w_i,dx_gust,gust_b] = dryden_wind(x_gust,V_a,dt,T_total,wind_steady,turbulance)
%INPUTS
%x_gust: vector5x1 con los estados de turbulencia
%V_a: airspeed
%z: altitud actual
%dt: paso de integracion
%T_total: matriz de rotacion
%wind_steady: viento constante [w_x;w_y;w_z]
%turbulance: string, 'light' o 'moderate'

%Parametros de escala de turbulencia
L_u = 200;
L_v = 200;
L_w = 50;
if turbulance == 'light'
    sigma_u = 1.06;
    sigma_v = 1.06;
    sigma_w = 0.7;
else
    sigma_u = 2.12;
    sigma_v = 2.12;
    sigma_w = 1.4;
end

a_u = V_a/L_u;
a_v = V_a/L_v;
a_w = V_a/L_w;

%Generacion del ruido
noise = randn(3,1)/sqrt(dt);

dx_gust = [
    -a_u*x_gust(1)+noise(1);
    x_gust(3);
    -a_v*x_gust(2)-2*a_v*x_gust(3)+noise(2);
    x_gust(5);
    -a_w^2*x_gust(4)-2*a_w*x_gust(5)+noise(3)
    ];

%gust en body frame (ojo) ojo que a esto le falta el runge kutta 
u_g = sigma_u * sqrt(2 * a_u / pi) * x_gust(1);
v_g = sigma_v * sqrt(3 * a_v / pi) * (x_gust(3) + (a_v / sqrt(3)) * x_gust(2));
w_g = sigma_w * sqrt(3 * a_w / pi) * (x_gust(5) + (a_w / sqrt(3)) * x_gust(4));
gust_b = [u_g; v_g; w_g];

%Transformación a marco inercial
gust_i = T_total * gust_b;

%salida   
v_w_i = wind_steady + gust_i;
end

