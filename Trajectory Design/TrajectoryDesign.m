%% Define and plot the trajectory 

T = 10;
w = 2*pi/T;

h = 1e-2;
t = 0:h:T;

x = sin(2*(w*t-pi/2));
y = 2*cos(w*t-pi/2);
z = 1.2 + 0.4*cos(2*w*t-pi);

vx = 2*w*cos(2*(w*t-pi/2));
vy = -2*w*sin(w*t-pi/2);
vz = -0.8*w*sin(2*w*t-pi);

ax = -4*w^2*sin(2*(w*t-pi/2));
ay = -2*w^2*cos(w*t-pi/2);
az = -1.6*w^2*cos(2*w*t-pi);

% Plot

figure
hold on
plot3(x,y,z,'LineWidth',1);
dt = T/20;
ds = 1:floor(dt/h):length(t);
quiver3(x(ds),y(ds),z(ds),ax(ds),ay(ds),az(ds)+9.81,0.3);
grid on
axis equal
xlim([-1.8,1.8]);
ylim([-3,3]);
zlim([0,2]);


%% Compute and plot the thrust norm over time, during the trajectory
m = 1.35;
Thrust = m*vecnorm(9.81*[0;0;1] + [ax;ay;az]);

figure
plot(t,Thrust,'LineWidth',1);
grid on

%% 
v_dir = vy./vx;
v_dir(abs(v_dir)>5) = 0;
figure
plot(t,v_dir)
figure
plot(t,atan2d(vy,vx))
%hold on
%plot(t,atand(vy./vx))