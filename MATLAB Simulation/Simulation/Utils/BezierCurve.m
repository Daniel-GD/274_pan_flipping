function u = BezierCurve(ctrl_pt, t)

n = length(ctrl_pt);
u = 0;
n1=n-1; 

for i = 1:n
    s=factorial(n1)/(factorial(i-1)*factorial(n-i))*t^(i-1)*(1-t)^(n-i)*ctrl_pt(i);
    u = u+ s; % compute return value. Write your code instead of 1.
end

end