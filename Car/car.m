function X_next = car(T, dotpsi_r, Vr, V, omega, X)
    xe = X(1); ye = X(2); psie = X(3);

    x = xe + T * (-V + Vr * cos(psie) + ye * omega);
    y = ye + T * (Vr * sin(psie) - xe * omega);
    psi = psie + T * (dotpsi_r - omega);

    X_next = [x; y; psi];  % Return as a column vector (3x1)
end
