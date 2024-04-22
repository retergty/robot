clear;

load("walkpattern.txt");

com = walkpattern(1:3,:);
left = walkpattern(4:6,:);
right = walkpattern(7:9,:);
ref_zmp =  walkpattern(10:11,:);

figure(1);
plot3(com(1,:),com(2,:),com(3,:));
hold on;
plot3(left(1,:),left(2,:),left(3,:));
hold on;
plot3(right(1,:),right(2,:),right(3,:));
grid on;