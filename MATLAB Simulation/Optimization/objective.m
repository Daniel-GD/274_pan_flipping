function f = objective(x,z0,p,dt,extra)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.
%
% Note: fmincon() requires a handle to an objective function that accepts 
% exactly one input, the decision variables 'x', and returns exactly one 
% output, the objective function value 'f'.  It is convenient for this 
% assignment to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().
    
    tol1=.01;
    
    %run simulation
    bezier_pts=3;
    tf=extra(1);
    ctrl.tf=extra(2);
%     ctrl.T1=x(1:bezier_pts); 
%     ctrl.T2=x(1+bezier_pts:end);    
    ctrl.T1 = extra(3:end);
    ctrl.T2 = x;
    
    % z0 is a struct with arm & pk
%     [arm, pk, contact_pts, tspan, uout]=simulate_system(z0, p, ctrl, tf, dt);
%     valid_idx = tspan < ctrl.tf;
%     u_pan = uout(1:2,1:sum(valid_idx)); % joint torques up until pancake leaves pan
%     f = sum(sum(u_pan.^2)); % minimize T^2 integral
    f=0;
    % alternate objective functions:
%     f = tf;                                         % final time
%     f = sum(uout.^2);                                         % minimize T^2 integral
    
end