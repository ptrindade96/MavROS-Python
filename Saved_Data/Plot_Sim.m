%% Load Data
clear
clc

files = dir("*Drone_*");
N = length(files);

for i=1:N
    load(files(i).name)
end

i = 1;
n = N;
while n>0
    if exist(["Drone_"+i],'var')
        Drones(i) = eval(["Drone_"+i]);
        l(i) = length(Drones(i).Time(Drones(i).Controlling>0));
        n = n - 1;
    end
    i = i + 1;
end

[~,m] = min(l);
Drones(1) = Drones(3);

%%
a=2.5;
h=1/40;
fade = fix(8/h);
figure('Position', [1920/2-1024/2 10 1024 1024])
xlim([-a a])
ylim([-a a])
hold on
grid on
for i=1:length(Drones)
    plot(Drones(i).Position(1,1),Drones(i).Position(2,1),'ob','LineWidth',2);
end
xlim([-a a])
ylim([-a a])
pause(0.5)

vec = 1:length(Drones(m).Time);
for i=1:N
    s = vec(:,Drones(i).Controlling>0);
    start(i) = s(1);
end
for i=1:l(m)-1
    hold off
    for j=1:length(Drones)
        R = 0.4;
        plot(Drones(j).Position(1,1),Drones(j).Position(2,1),'ob','LineWidth',2);
        hold on
        grid on
        plot(Drones(j).Position(1,end),Drones(j).Position(2,end),'xb','LineWidth',2);
        if i > fade
            plot(Drones(j).Position(1,i-fade+start(j):i+start(j)),Drones(j).Position(2,i-fade+start(j):i+start(j)),'-b','LineWidth',2);
        else
            plot(Drones(j).Position(1,start(j):i+start(j)),Drones(j).Position(2,start(j):i+start(j)),'-b','LineWidth',2);
        end
        circle = [([Drones(j).Position(1,i+start(j));Drones(j).Position(2,i+start(j))]-R).' 2*R*[1 1]];
        rectangle('Position',circle,'Curvature',[1 1],'FaceColor','none','EdgeColor','blue')
        x = Drones(j).Position(1,i+start(j));
        y = Drones(j).Position(2,i+start(j));
        xx = 0.5*cos(Drones(j).Yaw(i+start(j)));
        yy = 0.5*sin(Drones(j).Yaw(i+start(j)));
        quiver(x,y,xx,yy);
    end
  
    xlim([-a a])
    ylim([-a a])
    pause(h)
end

%%

a=2.5;
h=1/40;
fade = fix(15/h);
figure('Position', [1920/2-1024/2 10 1024 1024])
xlim([-a a])
ylim([-a a])
hold on
grid on
for i=1:length(Drones)
    plot3(Drones(i).Position(1,1),Drones(i).Position(2,1),Drones(i).Position(3,1),'ob','LineWidth',2);
end
xlim([-a a])
ylim([-a a])
pause(0.5)

vec = 1:length(Drones(m).Time);
for i=1:N
    s = vec(:,Drones(i).Controlling>0);
    start(i) = s(1);
end
for i=1:l(m)-1
    hold off
    for j=1:length(Drones)
        R = 0.4;
        plot3(Drones(j).Position(1,1),Drones(j).Position(2,1),Drones(j).Position(3,1),'ob','LineWidth',2);
        hold on
        grid on
        plot3(Drones(j).Position(1,end),Drones(j).Position(2,end),Drones(j).Position(3,end),'xb','LineWidth',2);
        if i > fade
            plot3(Drones(j).Position(1,i-fade+start(j):i+start(j)),Drones(j).Position(2,i-fade+start(j):i+start(j)),Drones(j).Position(3,i-fade+start(j):i+start(j)),'-b','LineWidth',2);
        else
            plot3(Drones(j).Position(1,start(j):i+start(j)),Drones(j).Position(2,start(j):i+start(j)),Drones(j).Position(3,start(j):i+start(j)),'-b','LineWidth',2);
        end
        plot3(Drones(j).Position(1,i+start(j)),Drones(j).Position(2,i+start(j)),Drones(j).Position(3,i+start(j)),'o','MarkerSize',15);
        circle = [([Drones(j).Position(1,i+start(j));Drones(j).Position(2,i+start(j))]-R).' 2*R*[1 1]];
        rectangle('Position',circle,'Curvature',[1 1],'FaceColor','none','EdgeColor','blue')
        x = Drones(j).Position(1,i+start(j));
        y = Drones(j).Position(2,i+start(j));
        xx = 0.5*cos(Drones(j).Yaw(i+start(j)));
        yy = 0.5*sin(Drones(j).Yaw(i+start(j)));
        quiver(x,y,xx,yy);
    end
    axis equal
    xlim([-1.8 1.8])
    ylim([-3 3])
    zlim([0 2])
    view(140,25);
    pause(h)
end