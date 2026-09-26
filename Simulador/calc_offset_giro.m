function C = calc_offset_giro(bank_rad, wn, dt)
% CALC_OFFSET_GIRO Calcula la constante C (independiente de V) tal que el
% ángulo de anticipación de salida de un viraje se obtiene como:
%
%   heading_offset(V) = C / V
%
%
%   dpsi/dt = g * tan(roll) / V
%

    g = 9.81;
    et = exp(-wn*dt);

    roll      = bank_rad;
    roll_rate = 0;
    acc       = 0;

    for k = 1:200000   % cota de seguridad; en la práctica converge en pocas decenas de pasos
        e0 = roll;             % target = 0
        v0 = roll_rate;
        e_new = (e0 + (v0 + wn*e0)*dt) * et;
        v_new = (v0 - wn*(v0 + wn*e0)*dt) * et;
        roll_rate = v_new;
        roll      = e_new;

        acc = acc + tan(roll) * dt;

        if abs(roll) < 1e-9
            break;
        end
    end

    C = g * acc;
end