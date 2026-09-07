% 对比图
problem1_shielding=1.41;
problem2_shielding=4.84;

figure;
bar([problem1_shielding,problem2_shielding]);
set(gca,'XTickLabel',{'固定策略','优化后策略'});
ylabel('累计遮蔽时长');
%title('遮蔽时长对比');
grid on;
