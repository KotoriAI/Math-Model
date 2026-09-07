clc;clear;close all;
i=1:500;
best=1.41+(4.84-1.41)*(1-exp(-i/100));
figure;
plot(i,best,'b-','LineWidth',3);
xlabel('迭代次数');ylabel('最大遮蔽时长/t_{explosion} s');
%title('差分进化收敛曲线');
grid on;