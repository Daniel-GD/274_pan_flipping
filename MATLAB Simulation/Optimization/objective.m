function f = objective(x,z0,p)
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
    
    f = 0;
%(m2*(y + c2*sin(th) + l*sin(th)) + m1*(y + c1*sin(th)) + mh*(y + 2*l*sin(th)))/(m1 + m2 + mh)
%     tf=x(1);
%     ctrl.tf=x(2);
%     ctrl.T=x(3:end);
%     tspan= [0 tf];
%     [tout, zout, uout, indices]=hybrid_simulation(z0,ctrl,p,tspan);
%     
    
%     COMs=COM_jumping_leg(zout,p);
%     f= -max(COMs(2,:));
%                                          %negative of COM height
    
    % alternate objective functions:
%     f = tf;                                         % final time
%     f = sum(uout.^2);                                         % minimize T^2 integral
    
end