% 无人机,目标，导弹，烟幕球坐标示意图
clc;clear;close all;

figure;
hold on; grid on; axis equal;
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');

target=[0,0,0];
scatter3(target(1),target(2),target(3),100,'bp','filled');

M1=[20000,0,2000];
t=linspace(0,1,100);
M=(1-t)'*M1+t'*target;
plot3(M(:,1),M(:,2),M(:,3),'r','LineWidth',2);
C0=[17898.309,12.168,1797.547];
scatter3(C0(1),C0(2),C0(3),100,'go','filled');
FY1=[17800,0,1800];
scatter3(FY1(1),FY1(2),FY1(3),100,'ms','filled');

plot3([C0(1),target(1)],[C0(2),target(2)],[C0(3),target(3)],'g--','LineWidth',1.5);
%title('无人机,目标，导弹，烟幕球坐标示意图');
legend('目标','导弹轨迹','干扰弹起爆点','FY1','烟幕球中心点-目标连线');
view(45,30);