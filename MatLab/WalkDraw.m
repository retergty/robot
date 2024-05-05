clear;

load("walkpattern3.txt");

com = walkpattern3(1:3,:);
left = walkpattern3(4:6,:);
right = walkpattern3(7:9,:);
ref_zmp =  walkpattern3(10:11,:);
zmp = walkpattern3(12:13,:);

fh = figure(1);
grid on;
for i=1:5:length(com)
    plot3(com(1,i),com(2,i),com(3,i),'LineWidth',20);
    hold on;
    xlim([-0.2 0.5]);
    ylim([-0.2 0.5]);
    zlim([-0.2 0.5]);
    
%     [X,Y,Z] = sphere(10);
%     for j=1:length(X)
%         for k = 1:length(X)
%             X(j,k) = X(j,k)/50 + com(1,i);
%             Y(j,k) = Y(j,k)/50 + com(2,i);
%             Z(j,k) = Z(j,k)/50 + com(3,i);
%         end
%     end
%     surf(X,Y,Z);
    plot3([com(1,i) left(1,i)],[com(2,i) left(2,i)],[com(3,i) left(3,i)],'LineWidth',5);
    plot3([com(1,i) right(1,i)],[com(2,i) right(2,i)],[com(3,i) right(3,i)],'LineWidth',5);
    plot3(zmp(1,i),zmp(2,i),0);
    drawnow();
    grid on;
    hold off;
    pause(0.01);
end
plot3(com(1,:),com(2,:),com(3,:));
hold on;
plot3(left(1,:),left(2,:),left(3,:));
hold on;
plot3(right(1,:),right(2,:),right(3,:));
hold on;
plot3(zmp(1,:),zmp(2,:),zeros(1,length(zmp)));
plot3(ref_zmp(1,:),ref_zmp(2,:),zeros(1,length(ref_zmp)));
grid on;