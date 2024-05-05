clear;
close all;
load("walkpattern4.txt");

com = walkpattern4(1:3,:);
left = walkpattern4(4:6,:);
right = walkpattern4(7:9,:);
ref_zmp =  walkpattern4(10:11,:);
zmp = walkpattern4(12:13,:);

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