clc; clear; close all;

theta=0.772814;
v_u=140.000;
t_drop=0;
delay=0.708;
t_explosion=t_drop+delay;
c_cover=[t_explosion,t_explosion+20];
t_start=3.439395;t_end=8.279395;
shielding=[t_start,t_end];
cover_time=shielding(2)-shielding(1);

figure;
hold on; grid on;
line([c_cover(1) c_cover(2)], [1 1], 'Color','b','LineWidth',8);
line([shielding(1) shielding(2)], [1 1], 'Color','r','LineWidth',8);
legend('烟幕球存在区间','有效遮蔽区间');
xlabel('时间'); 
yticks([]);
%title('无人机烟幕遮蔽时间区间');
