function C = calc_offset_giro(bank_rad, wn)
% CALC_OFFSET_GIRO Calcula la constante C (independiente de V) tal que el
% ángulo de anticipación de salida de un viraje se obtiene como:
%
%   heading_offset(V) = C / V
%
% Motivo físico: al salir del tramo "giro" el banqueo no cae a 0 de forma
% instantánea, sino que decae según el mismo filtro crítico de 2do orden
% que ya se usa para el roll (misma respuesta cerrada, asumiendo
% roll_rate ~= 0 en el momento de salir, válido si el viraje duró más de
% un par de veces t_transicion):
%
%   roll(t) = bank_rad * (1 + wn*t) * exp(-wn*t)
%
% Mientras decae, el avión sigue virando (viraje coordinado):
%
%   dpsi/dt = g * tan(roll(t)) / V
%
% Integrando en el tiempo, el rumbo "de más" que se acumula durante ese
% des-banqueo es:
%
%   offset(V) = (1/V) * g * integral( tan(roll(t)), t=0..inf ) = C / V
%
% Como roll(t) no depende de V, C = g*integral(tan(roll(t))dt) depende
% solo del ángulo de banco y de wn, y se calcula una sola vez por tramo
% (no en cada paso de integración).

    g = 9.81;
    t_max_int = 6 / wn;              % tiempo suficiente para que roll(t) decaiga a ~0
    t = linspace(0, t_max_int, 3000);
    roll_t = bank_rad * (1 + wn*t) .* exp(-wn*t);
    C = g * trapz(t, tan(roll_t));
end