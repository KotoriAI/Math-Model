clc; clear; close all;

false_target=[0,0,0];% 假目标,原点
true_target=[0,200,0];% 真目标

M=[20000,0,2000;   %三枚导弹位置
    19000,600,2100;
    18000,-600,1900];
FY=[17800,0,1800;   %五架无人机位置
    12000,1400,1400;
    6000,-3000,700;
    11000,2000,1800;
    13000,-2000,1300];

figure;% 创建3D图
hold on;grid on;axis equal;

plot3(false_target(1), false_target(2), false_target(3), ...% 假目标
      'm*', 'MarkerSize', 12, 'LineWidth', 2);
text(false_target(1), false_target(2), false_target(3), ...
     ' 假目标', 'Color', 'm');

plot3(true_target(1), true_target(2), true_target(3), ...% 真目标
      'bo', 'MarkerSize', 10, 'LineWidth', 2);
text(true_target(1), true_target(2), true_target(3), ...
     ' 真目标', 'Color', 'b','HorizontalAlignment', 'right');


for i = 1:3
    plot3(M(i,1),M(i,2),M(i,3),'ks','MarkerSize',8,'MarkerFaceColor','k'); %导弹位置和连线
    text(M(i,1),M(i,2),M(i,3), ['M',num2str(i)],'Color','k');
    plot3([false_target(1),M(i,1)],[false_target(2), M(i,2)], ...
          [false_target(3),M(i,3)],'r-','LineWidth',1.5);
end

for i = 1:5   % 无人机位置
    plot3(FY(i,1),FY(i,2),FY(i,3),'g.','MarkerSize', 20);
    text(FY(i,1),FY(i,2),FY(i,3), ['FY',num2str(i)],'Color','g');
end

xlabel('X'); ylabel('Y'); zlabel('Z');% 坐标轴设置
title('位置示意图');
view(3);
rotate3d on;
zlim([0 inf]);