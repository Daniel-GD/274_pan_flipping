function animate_system(arm, pk, contact_pts, p, tspan)
    close all
    z_out_arm=arm.z_out;
    z_out_pk=pk.z_out;
    p_arm=p.arm;
    p_pk=p.pk;

    %% Animate Solution
    figure; clf;
    % Prepare arm plot handles
    hold on
    h_link1 = plot([0],[0],'b','LineWidth',5);
    h_link2 = plot([0],[0],'b','LineWidth',5);
    h_B_mark = plot([0],[0],'.', 'MarkerSize' ,20,'MarkerEdgeColor','black');
    h_c1_mark = plot([0],[0],'.', 'MarkerSize' ,15,'MarkerEdgeColor','red');
    h_c2_mark = plot([0],[0],'.', 'MarkerSize' ,15,'MarkerEdgeColor','red');
    % Prepare pancake plot handle
    h_pk_line = plot([0],[0],'b','LineWidth',5);
    h_pk_line.Color=[181 101 30]./255;
    h_pk_com =plot([0],[0],'.', 'MarkerSize' ,20,'MarkerEdgeColor','black');
    
    % Prepare contact plot handles
    h_contact1 =plot([0],[0],'.', 'MarkerSize' ,20,'MarkerEdgeColor','red');
    h_contact2 =plot([0],[0],'.', 'MarkerSize' ,20,'MarkerEdgeColor','red');
    xlabel('x')
    ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 .2 -.2 .2]);
    skip_frame = 100;
    
    %Step through and update animation
    for i=1:numel(tspan)
        if mod(i, skip_frame)
            continue
        end
        % interpolate to get state at current time.
        t = tspan(i);
        z_arm=z_out_arm(:,i);
        z_pk=z_out_pk(:,i);
        
        arm_keypoints = keypoints_arm(z_arm,p_arm);
        pk_keypoints = keypoints_pancake(z_pk,p_pk);
        
        %Get arm keypoints
        rc1 = arm_keypoints(:,1); % position vector to the CoM of link 1
        rB = arm_keypoints(:,2); %position vector to point B
        rc2 = arm_keypoints(:,3); % position vector to the CoM of link 2
        rC = arm_keypoints(:,4); %position vector to point C
        
        %Get pancake keypoints 
        rc_pk = pk_keypoints(:,1); % position vector to the CoM of pancake
        rA_pk = pk_keypoints(:,2); %position vector to point A in pancake
        rB_pk = pk_keypoints(:,3); % position vector to point B in pancake

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_link1,'XData',[0; rB(1)]);
        set(h_link1,'YData',[0; rB(2)]);
        
        set(h_link2,'XData',[rB(1); rC(1)]);
        set(h_link2,'YData',[rB(2); rC(2)]);
        
        set(h_B_mark,'XData',[rB(1)]);
        set(h_B_mark,'YData',[rB(2)]);
        
        set(h_c1_mark,'XData',[rc1(1)]);
        set(h_c1_mark,'YData',[rc1(2)]);
        
        set(h_c2_mark,'XData',[rc2(1)]);
        set(h_c2_mark,'YData',[rc2(2)]);
        
        %Plot contact point (NaN doesnt plot anything)
        set(h_contact1,'XData',[contact_pts(1,i)]);
        set(h_contact1,'YData',[contact_pts(2,i)]);
        
        set(h_contact2,'XData',[contact_pts(3,i)]);
        set(h_contact2,'YData',[contact_pts(4,i)]);
        
        
        %Set pancakes
        set(h_pk_line,'XData',[rA_pk(1); rB_pk(1)]);
        set(h_pk_line,'YData',[rA_pk(2); rB_pk(2)]);
        
        set(h_pk_com,'XData',[rc_pk(1)]);
        set(h_pk_com,'YData',[rc_pk(2)]);
%         break
        pause(.01)
    end
end

% function draw_arm(z,p, u)
%   
% end
