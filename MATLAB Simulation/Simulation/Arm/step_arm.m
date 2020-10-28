function z_out = step_arm(z0,p, u, dt)
    %% %% Performs one Forward Euler Integration on Arm
    dz = dynamics(z0, p, u);
    z_out = z0 + dz*dt;
    
end

function dz = dynamics(z,p, u)
    % Get mass matrix
    A = A_arm(z,p);
    
    % Get forces
    b = b_arm(z,u, p);
    
    % Solve for qdd
    qdd = A\b;
    dz = 0*z;
    
    % Form dz
    dz(1:2) = z(3:4);
    dz(3:4) = qdd;
end
