function [Xdot, thetades_all, omegades_all] = ...
              sim2dfunc(t, X, ...
                        g, m, I, ...
                        xdes, xdotdes, ...
                        attOnly, force_att, theta_att, thetadot_att, ...
                        kx, kxdot, kR, komega)
    
    R = [cos(X(5)) -sin(X(5)); sin(X(5)) cos(X(5))];
    Rdes = zeros(2,2);
    
    % control law
    if ~attOnly
        e_x    = xdes    - [1 0 0 0 0 0; 0 1 0 0 0 0] * X;
        e_xdot = xdotdes - [0 0 1 0 0 0; 0 0 0 1 0 0] * X;

        force_x = kx * e_x;
        force_xdot = kxdot * e_xdot;
        forcedes = force_x + force_xdot + [0; 1] * m * g;
        thetades = atan(-forcedes(1) / forcedes(2));
        Rdes(:, 2) = forcedes ./ norm(forcedes);
        
        % derivative of (thetades = arctan of forcedes)
        % TODO(mereweth) - add setpoint feedforward into omega desired
        omegades = 1.0 / (1 + forcedes(1)^2 / forcedes(2)^2) ...
                   * (kx(1,1) * X(3) * forcedes(2) - kx(2, 2) * X(4) * forcedes(1)) ...
                   / forcedes(2)^2;
               
        force = R(:, 2) .* forcedes;
    else
        Rdes(:, 2) = [-sin(theta_att); cos(theta_att)];
        thetades = theta_att;
        omegades = thetadot_att;
        
        force = force_att;
    end
    Rdes(1, 1) =  Rdes(2, 2);
    Rdes(2, 1) = -Rdes(1, 2);
    
    e_R_mat = 0.5 * (R' * Rdes - Rdes' * R);
    e_R = e_R_mat(2, 1);

    e_omega = omegades - X(6);
    momdes = kR * e_R + komega * e_omega;
    
    % plant dynamics
    xddot = force / m .* R(:, 2) - [0; g];
    thetaddot = momdes / I;
    Xdot = [0 0 1 0 0 0; 0 0 0 1 0 0; ...
            zeros(1, 6); zeros(1, 6); ...
            0 0 0 0 0 1; ...
            zeros(1, 6)] * X ...
            + [0; 0; xddot; 0; 0] ...
            + [zeros(5, 1); thetaddot];
        
    if nargout > 1
        thetades_all = thetades;
        omegades_all = omegades;
        return
    end
end
