function simulate_arm3()
    close all
    %% Definite fixed paramters
    l1 = .5;
    l2 = .5;
    l3 = .3;
    c1= .6*l1;
    c2=.6*l2;
    c3=.8*l3;
    m1 = .8;
    m2 = .8;
    m3 = 1.3;
    I1=.005;
    I2 = I1;
    I3 = I2;
    k = 50;
    kappa = 10;
    th1_0 = pi/2;
    th2_0 = 0;
    th3_0 = 0;
    g = 9.81;
    
    T1_0=0;
    T2_0=0;
    T3_0=0;
    u0= [T1_0 T2_0 T3_0]';%[1.4e-5 0]';

    p   =  [c1; c2; c3; l1; l2; l3; m1; m2; m3; I1; I2; I3; k; kappa; th1_0; th2_0; th3_0; g];         % parameters

    %% Perform Dynamic simulation    
    dt = 0.00001;
    tf = 10;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    z0 = [th1_0; th2_0; th3_0; 0; 0; 0];  %initial values
    z_out = zeros(6,num_steps);
    z_out(:,1) = z0;
    for i=1:num_steps-1
        dz = dynamics(z_out(:,i), p, u0);
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
    title('Joint Positions vs Time')
    xlabel('Time (s)'); ylabel('Joint Angle (rads)');
    legend('theta_1','theta_2')
    
    
    %% Compute Energy
    E = energy_arm3(z_out,p);
    T = kinetic_energy_arm3(z_out,p);
    V = potential_energy_arm3(z_out,p);
    figure(2);
    title('Total Energy vs Time')
    plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');   
    

    %% Animate Solution
    figure(3); clf;
    % Prepare plot handles
    hold on
    h_link1 = plot([0],[0],'g','LineWidth',5);
    h_link2 = plot([0],[0],'g','LineWidth',5);
    h_link3 = plot([0],[0],'k','LineWidth',5);
    h_A_mark = plot([0],[0],'.', 'MarkerSize' ,20,'MarkerEdgeColor','black');
    h_B_mark = plot([0],[0],'.', 'MarkerSize' ,20,'MarkerEdgeColor','black');
    h_c1_mark = plot([0],[0],'.', 'MarkerSize' ,15,'MarkerEdgeColor','red');
    h_c2_mark = plot([0],[0],'.', 'MarkerSize' ,15,'MarkerEdgeColor','red');
    h_c3_mark = plot([0],[0],'.', 'MarkerSize' ,15,'MarkerEdgeColor','red');
    xlabel('x')
    ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-(l1+l2+l3) (l1+l2+l3) -(l1+l2+l3) (l1+l2+l3) ]);
    skip_frame = 1000;
    
    %Step through and update animation
    for i=1:num_steps
        if mod(i, skip_frame)
            continue
        end
        % interpolate to get state at current time.
        t = tspan(i);
        z = z_out(:,i);
        keypoints = keypoints_arm3(z,p);
        
        rc1 = keypoints(:,1); % position vector to the CoM of link 1
        rB = keypoints(:,2); %position vector to point B
        rc2 = keypoints(:,3); % position vector to the CoM of link 2
        rC = keypoints(:,4); %position vector to point C
        rc3 = keypoints(:,5); % position vector to the CoM of link 3
        rD = keypoints(:,6); %position vector to point D
        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_link1,'XData',[0; rB(1)]);
        set(h_link1,'YData',[0; rB(2)]);
        
        set(h_link2,'XData',[rB(1); rC(1)]);
        set(h_link2,'YData',[rB(2); rC(2)]);
        
        set(h_link3,'XData',[rC(1); rD(1)]);
        set(h_link3,'YData',[rC(2); rD(2)]);

        set(h_A_mark,'XData',[rB(1)]);
        set(h_A_mark,'YData',[rB(2)]);
        
        set(h_B_mark,'XData',[rB(1)]);
        set(h_B_mark,'YData',[rB(2)]);
        
        set(h_c1_mark,'XData',[rc1(1)]);
        set(h_c1_mark,'YData',[rc1(2)]);
        
        set(h_c2_mark,'XData',[rc2(1)]);
        set(h_c2_mark,'YData',[rc2(2)]);
        
        set(h_c3_mark,'XData',[rc3(1)]);
        set(h_c3_mark,'YData',[rc3(2)]);
        
        pause(.01)
    end
end

function dz = dynamics(z,p, u)
    % Get mass matrix
    A = A_arm3(z,p);
    
    % Get forces
%     u = [0 0]';
%     u=u;
%     u = [ 0 -5*z(4)]';
%     u = [ 0 1]';
    b = b_arm3(z,u, p);
    
    % Solve for qdd
    qdd = A\b;
    dz = 0*z;
    
    % Form dz
    dz(1:3) = z(4:6);
    dz(4:6) = qdd;
end
