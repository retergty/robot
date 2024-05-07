clear;
close all;
load("walkpattern3.txt");

com = walkpattern3(1:3,:);
left = walkpattern3(4:6,:);
right = walkpattern3(7:9,:);
ref_zmp =  walkpattern3(10:11,:);
zmp = walkpattern3(12:13,:);
t = zeros(1,length(com));
for i=1:length(t)
    t(i)= i*0.001;
end

figure(1);
plot3(com(1,:),com(2,:),com(3,:));
hold on;
plot3(left(1,:),left(2,:),left(3,:));
hold on;
plot3(right(1,:),right(2,:),right(3,:));
hold on;
plot3(zmp(1,:),zmp(2,:),zeros(1,length(zmp)));
plot3(ref_zmp(1,:),ref_zmp(2,:),zeros(1,length(ref_zmp)));
grid on;

figure(2);
plot(t,com(1,:));
hold on;
plot(t,com(2,:));
hold on;
plot(t,com(3,:));
hold on;
grid on;