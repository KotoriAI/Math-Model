% problem3 三枚烟幕弹遮蔽区间示例(单位：s)
clc;clear;close all;
cover_interval=[4.77 7.40;   % 三枚干扰弹的有效遮蔽区间
                  1.00 5.47;
                  0 0];
total_cover=[1.00 7.40];      % 总遮蔽时间区间

figure;
hold on;
colors=lines(3);
for i=1:3
    rectangle('Position',[cover_interval(i,1),i-0.4,...
              cover_interval(i,2)-cover_interval(i,1),0.8],...
              'FaceColor',colors(i,:),'EdgeColor','k');
end
rectangle('Position',[total_cover(1), 3.6, ...
          total_cover(2)-total_cover(1), 0.8], ...
          'FaceColor',[0.6 0.6 0.6], 'EdgeColor','k');
yticks(1:4);
yticklabels({'干扰弹1','干扰弹2','干扰弹3','总遮蔽区间'});
xlabel('时间(s)');
%title('三枚烟幕干扰弹遮蔽区间时间轴图');
grid on;