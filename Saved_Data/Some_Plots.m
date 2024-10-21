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

%% Plot position
subplot(2,3,[1 2 3])
hold on
grid minor
plot(Drones(7).Time-Drones(7).Time(1),Drones(7).Position,'LineWidth',1)

%% Plot velocity
subplot(2,3,[4 5 6])
hold on
grid minor
plot(Drones(7).Time-Drones(7).Time(1),Drones(7).Velocity,'LineWidth',1)