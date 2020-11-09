function [cineq ceq] = constraints(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().
    tf=x(1);
    ctrl.tf=x(2);
    ctrl.T=x(3:end);
    tspan= [0 tf];
    [tout, zout,~, indices]=hybrid_simulation(z0,ctrl,p,tspan);
    
    COMs=COM_jumping_leg(zout,p);
    f= max(COMs(2,:));
    apex_c= .4-f;
    
    cineq1=-min(zout(2,:));
    cineq2 =max(zout(2,:))-pi/2;
    
    cineq = [cineq1 , cineq2]; 
%     cineq = [cineq1 , cineq2, apex_c];  

%     if indices(1)==0 %Check if you actually took off
%         indices(1)=1;
%     end
    takeoff_ceq=ctrl.tf-tout(indices(1));
         ceq = [ takeoff_ceq];  
    
    ceq = [ takeoff_ceq apex_c]; 
    
                                                            
% simply comment out any alternate constraints when not in use
    
end