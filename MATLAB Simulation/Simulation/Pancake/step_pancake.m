function z_out=step_pancake(z0,p, u, dt)
    %% Performs one Forward Euler Integration on Pancake
    dz = dynamics(z0, p, u);
    z_out = z0 + dz*dt;
end

function dz = dynamics(z,p, u)
    % Get mass matrix
    A = A_pancake(z,p);
    
    % Get forces
%     u=[0; 0; 0];
    b = b_pancake(z,u, p);
    
    % Solve for qdd
    qdd = A\b;
    dz = 0*z;
    
    % Form dz
    dz(1:3) = z(4:6);
    dz(4:6) = qdd;
end