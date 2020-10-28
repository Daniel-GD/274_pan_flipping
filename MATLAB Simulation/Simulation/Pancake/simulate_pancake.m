function simulate_pancake()
    close all
    %% Definite fixed paramters
    l=.05;
    m=.005;
    I=.0001;
    x0=1;
    y0=1;
    th0=0;
    g=0.001;
%     g=9.81;
    
    dx0=0;
    dy0=0;
    dth0=0;
    
    Fx=0;
    Fy=0;
    Tau=0;
    
%     Fx=0.001;
    Fy=.001;
%     Tau=0.001;
    u=[Fx; Fy; Tau]; 

    p=[l; m; I; x0; y0; th0; g];

    %% Perform Dynamic simulation    
    dt = 0.00001;
    tf = 2;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    z0 = [x0; y0; th0; dx0; dy0; dth0];  %initial values
    keypoints_pancake(z0,p)
    z_out = zeros(6,num_steps);
    z_out(:,1) = z0;
    for i=1:num_steps-1
        dz = dynamics(z_out(:,i), p, u);
        z_out(:,i+1) = z_out(:,i) + dz*dt;
%         z_out(3:4,i+1) = z_out(3:4,i) + dz(3:4)*dt;
%         z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:4,i+1)*dt; % + 0.5*dz(3:4)*dt*dt;
    end
    final_state = z_out(:,end);
    

    %% Plot Solution
    figure(1)
    plot(tspan,z_out(1,:))
    hold on
    plot(tspan,z_out(2,:))
    plot(tspan,z_out(3,:))
    
    title('Pose vs Time')
    xlabel('Time (s)'); ylabel('Joint Angle (rads)');
    legend('x','y')
    
    
    %% Compute Energy
    E = energy_pancake(z_out,p);
    T = kinetic_energy_pancake(z_out,p);
    V = potential_energy_pancake(z_out,p);
    figure(2);
    title('Total Energy vs Time')
    plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');   
    

    %% Animate Solution
    figure(3); clf;
    % Prepare plot handles
    hold on
    h_link1 = plot([0],[0],'b','LineWidth',5);
    h_link1.Color=[181 101 30]./255;
    xlabel('x')
    ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    k_zoom=6;
    axis([(x0-l*k_zoom) (x0+l*k_zoom) (y0-l*k_zoom) (y0+l*k_zoom)]);
    skip_frame = 1000;
    
    %Step through and update animation
    for i=1:num_steps
        if mod(i, skip_frame)
            continue
        end
        % interpolate to get state at current time.
        t = tspan(i);
        z = z_out(:,i);
        keypoints = keypoints_pancake(z,p);
        
        rc = keypoints(:,1); % position vector to the CoM of link 1
        rA = keypoints(:,2); %position vector to point B
        rB = keypoints(:,3); % position vector to the CoM of link 2

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_link1,'XData',[rA(1); rB(1)]);
        set(h_link1,'YData',[rA(2); rB(2)]);
        
        pause(.01)
    end
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
