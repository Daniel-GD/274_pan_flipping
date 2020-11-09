clear
close all
pan=[0 0; 1 0];
% pk= [.1 .1;.5 0.05];
pk= [.1 0;.5 0];
% pk= [.5 .1;.7 -.1];
% pk= [.5 .1;.7 -.1];


x_pan=pan(:,1);
y_pan=pan(:,2);
x_pk=pk(:,1);
y_pk=pk(:,2);


%Find intersection

% [xi,yi] = polyxpoly(x1,y1,x2,y2)
[xi,yi] = polyxpoly(x_pan,y_pan,x_pk,y_pk)
figure
plot(x_pan,y_pan)
hold on
plot(x_pk,y_pk)
numel(xi(:,1))

plot(xi,yi,'or')



% [xi,yi] = polyxpoly(x1,y1,x2,y2)

