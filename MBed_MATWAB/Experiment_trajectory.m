function output_data = Experiment_trajectory( angle1_init, angle2_init, shoulder, elbow, torque_pts, traj_time, pre_buffer_time, post_buffer_time, gains, duty_max)
    
    % Figure for plotting motor data
    figure(1);  clf;       
    a1 = subplot(421);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');
    
    a2 = subplot(423);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)');
    
    a3 = subplot(425);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    subplot(425);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(427);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Cycle 1');
    

    a5 = subplot(422);
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)');
    
    a6 = subplot(424);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity 2 (rad/s)');
    
    a7 = subplot(426);
    h23 = plot([0],[0]);
    h23.XData = []; h23.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(426);
    h24 = plot([0],[0],'r');
    h24.XData = []; h24.YData = [];
    hold off;
    
    a8 = subplot(428);
    h25 = plot([0],[0]);
    h25.XData = []; h25.YData = [];
    ylabel('Duty Cycle 2');
    
    % Figure for plotting state of the leg
    figure(2)
    clf
    hold on
    axis equal
    axis([-.3 .1 -.3 .15]);
    title('Flip Trajectory')
%     yline(0)
    h_BC = plot([0],[0],'LineWidth',2);
    h_AB = plot([0],[0],'b-','LineWidth',5);
    h_BC = plot([0],[0],'b-','LineWidth',3);
    h_c1 = plot([0],[0],'ro','MarkerSize',7);
    h_c2 = plot([0],[0],'ro','MarkerSize',5);
    h_AB = plot([0],[0],'r','LineWidth',5);
    h_BC = plot([0],[0],'r','LineWidth',5);
    h_pan = plot([0],[0],'k','LineWidth',6);
    h_c1 = plot([0],[0],'ro','MarkerSize',6);
    h_c2 = plot([0],[0],'ro','MarkerSize',6);
    h_foot= plot([0],[0],'k');
%     h_des = plot([0],[0],'k--');
%     h_des.XData=[];
%     h_des.YData=[];
    h_foot.XData=[];
    h_foot.YData=[];
    
    p   = parameters();
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time
        pos1 = new_data(:,2);       % position
        vel1 = new_data(:,3);       % velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command
        
        pos2 = new_data(:,7);       % position
        vel2 = new_data(:,8);       % velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command
        
        x = -new_data(:,12);         % actual foot position (negative due to direction motors are mounted)
        y = new_data(:,13);         % actual foot position
        tau1 = new_data(:,16);      % desired foot position (negative due to direction motors are mounted)
        tau2 = new_data(:,17);      % desired foot position        
        tau1_des = new_data(:,18);      % desired foot position (negative due to direction motors are mounted)
        tau2_des = new_data(:,19);      % desired foot position         
        
        N = length(pos1);
        
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = -pos1; % switch sign on all plotted values due to direction motors are mounted
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = -vel1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = -cur1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = -dcur1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = -duty1;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = -pos2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = -vel2;
        h23.XData(end+1:end+N) = t;   
        h23.YData(end+1:end+N) = -cur2;
        h24.XData(end+1:end+N) = t;   
        h24.YData(end+1:end+N) = -dcur2;
        h25.XData(end+1:end+N) = t;   
        h25.YData(end+1:end+N) = -duty2;
        
        % Calculate leg state and update plots
        z = [pos1(end) pos2(end) vel1(end) vel2(end)]';
        keypoints = keypoints_arm(z,p.arm);
        
        % TODO: add code here to show Jacobian and/or mass matrix as
        % ellipses, controller force vectors and desired force vectors?
        
        rc1 = keypoints(:,1); 
        rB = keypoints(:,2);
        rc2 = keypoints(:,3);
        rpA = keypoints(:,4);
        rC = keypoints(:,5);
        
        set(h_AB,'XData',[0 rB(1)],'YData',[0 rB(2)]);
        set(h_BC,'XData',[rB(1) rC(1)],'YData',[rB(2) rC(2)]);
        set(h_pan,'XData',[rpA(1) rC(1)],'YData',[rpA(2) rC(2)]);
        set(h_c1,'XData',[rc1(1)],'YData',[rc1(2)]);
        set(h_c2,'XData',[rc2(1)],'YData',[rc2(2)]);
        
        h_foot.XData(end+1:end+N) = -x;
        h_foot.YData(end+1:end+N) = y;
%         h_des.XData(end+1:end+N) = tau1_des;
%         h_des.YData(end+1:end+N) = tau2_des;
        
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
%     start_period                = pre_buffer_time;    % In seconds 
%     end_period                  = post_buffer_time;   % In seconds
    
    K_xx                     = gains.K_xx; % Stiffness
    K_yy                     = gains.K_yy; % Stiffness
    K_xy                     = gains.K_xy; % Stiffness

    D_xx                     = gains.D_xx; % Damping
    D_yy                     = gains.D_yy; % Damping
    D_xy                     = gains.D_xy; % Damping
    
    % Specify inputs                             % in MBed, indeces:
    input = [pre_buffer_time traj_time post_buffer_time]; % 0 1 2
    input = [input angle1_init angle2_init shoulder elbow];     % 3 4 5 6
    input = [input K_xx K_yy K_xy D_xx D_yy D_xy]; % 7 8 9 10 11 12
    input = [input duty_max]; % 13
    input = [input p.arm(3) p.arm(4)]; %Add arm parameters %14 15
    input = [input torque_pts(:)']; % final size of input should be 28x1 16:end
    
    params.timeout  = (pre_buffer_time+traj_time+post_buffer_time);  
    
    output_size = 21;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2 a3 a4],'x')
    
end