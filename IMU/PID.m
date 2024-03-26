function [u, e_int] = PID(e, e_int, P, I, D, dt)
    e_dot = e / dt;
    e_int = e_int + e * dt;

    u = P * e + I * e_int + D * e_dot;
end