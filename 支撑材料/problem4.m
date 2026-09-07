%% 注：该脚本运行时间约在16分钟左右！！！

clear;clc;rng(0);

%% 常量参数
% 各点坐标
M1=[20000,0,2000];    
V_missile=300;    
fake_target=[0,0,0];
true_target=[0,200,5];    %取真目标中心高度5
FY1=[17800,0,1800];
FY2=[12000,1400,1400];
FY3=[6000,-3000,700];
g=9.8;         % 重力加速度
R=10;          % 烟幕球半径
c_down=3;      % 烟幕下沉速度
c_cover=20;    % 烟幕有效时间
dt=0.01;       % 时间仿真步长
t_boom=norm(M1-fake_target)/V_missile;% 导弹命中假目标所需时间

%% 决策变量边界
% 水平航向角(rad),0度为x轴正方向,逆时针
Theta_min=0;
Theta_max=2*pi;
v_min=70;v_max=140; %无人机速度范围
t_drop_min=0;       %投放时间范围
t_drop_max=t_boom;
delay_min=0;delay_max=c_cover;        % 起爆延时不大于烟幕球存在时间

%% 差分进化参数
NP=200;        %种群规模大小
Gmax=5000;     %最多迭到第几代
F=0.8;         %缩放因子大小
CR=0.9;        %交叉的概率

%% 初始化种群
D = 12;
po = zeros(NP, D);
for i = 1:NP
    %FY1
    po(i,1)  = Theta_min + (Theta_max-Theta_min)*rand();    % 航向角theta
    po(i,2)  = v_min + (v_max-v_min)*rand();                % 无人机速度v
    po(i,3)  = t_drop_min + (t_drop_max-t_drop_min)*rand(); % 投弹时间t_drop
    po(i,4)  = delay_min + (delay_max-delay_min)*rand();    % 延时引爆delay
    %FY2
    po(i,5)  = Theta_min + (Theta_max-Theta_min)*rand();    
    po(i,6)  = v_min + (v_max-v_min)*rand();                
    po(i,7)  = t_drop_min + (t_drop_max-t_drop_min)*rand(); 
    po(i,8)  = delay_min + (delay_max-delay_min)*rand();    
    %FY
    po(i,9)  = Theta_min + (Theta_max-Theta_min)*rand();    
    po(i,10) = v_min + (v_max-v_min)*rand();                
    po(i,11) = t_drop_min + (t_drop_max-t_drop_min)*rand(); 
    po(i,12) = delay_min + (delay_max-delay_min)*rand();    
end

% 评估初代适应度
fitness = zeros(NP,1);
for i=1:NP
    fitness(i)=fitness_cover(po(i,:),M1,V_missile,fake_target,true_target,...
    FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom,...
    Theta_min,Theta_max,v_min,v_max,t_drop_min,t_drop_max,delay_min,delay_max);
end

%% 差分进化主循环
best_f=-inf;
best_x=[];
for gen=1:Gmax
    for i=1:NP
        idx=1:NP;idx(i)=[];
        r=idx(randperm(NP-1,3));
        xr1=po(r(1),:);xr2=po(r(2),:);xr3=po(r(3),:);
        v_vec=xr1+F*(xr2-xr3);
        u=po(i,:);
        jrand = randi(D);
        for j=1:D
            if rand<=CR||j==jrand
                u(j)=v_vec(j);
            end
        end
        
        u(1)=mod(u(1),2*pi);            %处理边界
        u(2)=max(v_min,min(v_max,u(2)));
        u(3)=max(t_drop_min,min(t_drop_max,u(3)));
        u(4)=max(delay_min,min(delay_max,u(4)));
        u(5)=mod(u(5),2*pi);
        u(6)=max(v_min,min(v_max,u(6)));
        u(7)=max(t_drop_min,min(t_drop_max,u(7)));
        u(8)=max(delay_min,min(delay_max,u(8)));
        u(9)=mod(u(9),2*pi);
        u(10)=max(v_min,min(v_max,u(10)));
        u(11)=max(t_drop_min,min(t_drop_max,u(11)));
        u(12)=max(delay_min,min(delay_max,u(12)));
        
        % 计算适应度
        f=fitness_cover(u,M1,V_missile,fake_target,true_target,...
            FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom,...
            Theta_min,Theta_max,v_min,v_max,t_drop_min,t_drop_max,delay_min,delay_max);
        
        if f>fitness(i)
            po(i,:)=u;
            fitness(i)=f;
        end
    end
    
    [g_best_f,idx_best]=max(fitness);% 更新最优
    if g_best_f>best_f
        best_f=g_best_f;
        best_x=po(idx_best,:);
    end

    % 每十代打印当前结果
    if mod(gen,10)==0
        fprintf('第%d/%d代:当前最长遮挡时长为%.2fs\n',gen,Gmax,best_f);
    end
end

%% 打印最优解
fprintf('\nProblem4\n');
fprintf('最大遮蔽时长为%.2fs\n', best_f);

Theta1 = best_x(1); v1 = best_x(2); t_drop1 = best_x(3); delay1 = best_x(4);
Theta2 = best_x(5); v2 = best_x(6); t_drop2 = best_x(7); delay2 = best_x(8);
Theta3 = best_x(9); v3 = best_x(10); t_drop3 = best_x(11); delay3 = best_x(12);

fprintf('FY1最优参数：航向角为%.3f(deg),无人机速度为%.3fm/s,投放时间为%.2fs,投放间隔%.2fs\n', Theta1*180/pi, v1, t_drop1, delay1);
fprintf('FY2最优参数：航向角为%.3f(deg),无人机速度为%.3fm/s,投放时间为%.2fs,投放间隔%.2fs\n', Theta2*180/pi, v2, t_drop2, delay2);
fprintf('FY3最优参数：航向角为%.3f(deg),无人机速度为%.3fm/s,投放时间为%.2fs,投放间隔%.2fs\n', Theta3*180/pi, v3, t_drop3, delay3);

% 计算并打印投放点和起爆点以及总遮蔽区间
d1=[cos(Theta1),sin(Theta1),0];
d2=[cos(Theta2),sin(Theta2),0];
d3=[cos(Theta3),sin(Theta3),0];

drop1=FY1+d1*v1*t_drop1;
drop2=FY2+d2*v2*t_drop2;
drop3=FY3+d3*v3*t_drop3;

explosion1=[drop1(1)+v1*d1(1)*delay1,drop1(2)+v1*d1(2)*delay1,drop1(3)-0.5*g*delay1^2];
explosion2=[drop2(1)+v2*d2(1)*delay2,drop2(2)+v2*d2(2)*delay2,drop2(3)-0.5*g*delay2^2];
explosion3=[drop3(1)+v3*d3(1)*delay3,drop3(2)+v3*d3(2)*delay3,drop3(3)-0.5*g*delay3^2];

if explosion1(3)<0,explosion1(3)=0;end   %若z<0则剪枝
if explosion2(3)<0,explosion2(3)=0;end
if explosion3(3)<0,explosion3(3)=0;end

t_explosion1=t_drop1+delay1;
t_explosion2=t_drop2+delay2;
t_explosion3=t_drop3+delay3;

fprintf('干扰弹1投放坐标(%.3f,%.3f,%.3f),投放时间为%.2fs,爆炸时间为=%.2fs,爆炸坐标为(%.3f,%.3f,%.3f)\n', ...
    drop1, t_drop1, t_explosion1, explosion1);
fprintf('干扰弹1投放坐标(%.3f,%.3f,%.3f),投放时间为%.2fs,爆炸时间为=%.2fs,爆炸坐标为(%.3f,%.3f,%.3f)\n', ...
    drop2, t_drop2, t_explosion2, explosion2);
fprintf('干扰弹1投放坐标(%.3f,%.3f,%.3f),投放时间为%.2fs,爆炸时间为=%.2fs,爆炸坐标为(%.3f,%.3f,%.3f)\n', ...
    drop3, t_drop3, t_explosion3, explosion3);

% 计算总体遮蔽区间
[cover_time,intervals]=cover_intervals(best_x,M1,V_missile,fake_target,true_target,...
    FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom);
if isempty(intervals)
    fprintf('未检测到总体遮蔽区间\n');
else
    fprintf('检测到%d个总体遮蔽区间：\n', size(intervals,1));
    for k=1:size(intervals,1)
        fprintf('区间%d: %.2fs~%.2fs (时长=%.2fs)\n',k,intervals(k,1),intervals(k,2),intervals(k,2)-intervals(k,1)+dt);
    end
end

% 为每枚弹单独计算并打印其遮蔽区间与时长
[ct1,ints1]=single_bomb_interval(t_explosion1,explosion1,1,best_x,M1,V_missile,fake_target,true_target,...
    FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom);
[ct2,ints2]=single_bomb_interval(t_explosion2,explosion2,2,best_x,M1,V_missile,fake_target,true_target,...
    FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom);
[ct3,ints3]=single_bomb_interval(t_explosion3,explosion3,3,best_x,M1,V_missile,fake_target,true_target,...
    FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom);

fprintf('干扰弹1单独遮蔽时长为%.2fs\n',ct1);
if ~isempty(ints1)
    for kk=1:size(ints1,1)
        fprintf('%.2fs~%.2fs,时长为%.2fs\n',ints1(kk,1),ints1(kk,2),ints1(kk,2)-ints1(kk,1)+dt);
    end
else
    fprintf('干扰弹1无遮蔽段\n');
end

fprintf('干扰弹2单独遮蔽时长为%.2fs\n',ct2);
if ~isempty(ints2)
    for kk=1:size(ints2,1)
        fprintf('%.2fs~%.2fs,时长为%.2fs\n',ints2(kk,1),ints2(kk,2),ints2(kk,2)-ints2(kk,1)+dt);
    end
else
    fprintf('干扰弹2无遮蔽段\n');
end

fprintf('干扰弹3单独遮蔽时长为%.2fs\n',ct3);
if ~isempty(ints3)
    for kk=1:size(ints3,1)
        fprintf('%.2fs~%.2fs,时长为%.2fs\n',ints3(kk,1),ints3(kk,2),ints3(kk,2)-ints3(kk,1)+dt);
    end
else
    fprintf('干扰弹3无遮蔽段\n');
end

%% 目标函数
function cover_time=fitness_cover(x,M1,V_missile,fake_target,true_target,...
    FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom,...
    Theta_min,Theta_max,v_min,v_max,t_drop_min,t_drop_max,delay_min,delay_max)

    Theta1=mod(x(1),2*pi);v1=max(v_min,min(v_max,x(2)));t1=max(t_drop_min,min(t_drop_max,x(3)));delay1=max(delay_min,min(delay_max,x(4)));
    Theta2=mod(x(5),2*pi);v2=max(v_min,min(v_max,x(6)));t2=max(t_drop_min,min(t_drop_max,x(7)));delay2=max(delay_min,min(delay_max,x(8)));
    Theta3=mod(x(9),2*pi);v3=max(v_min,min(v_max,x(10)));t3=max(t_drop_min,min(t_drop_max,x(11)));delay3=max(delay_min,min(delay_max,x(12)));
    
    
    t_explosion1=t1+delay1;t_ex2=t2+delay2;t_ex3=t3+delay3;%计算起爆时间
    
    if t_explosion1>t_boom&&t_ex2>t_boom&&t_ex3>t_boom%如果起爆时间晚于导弹命中时间剪枝
        cover_time=0;return;
    end
    
    %计算投放点与爆炸点
    d1=[cos(Theta1),sin(Theta1),0];drop1=FY1+d1*v1*t1;
    d2=[cos(Theta2),sin(Theta2),0];drop2=FY2+d2*v2*t2;
    d3=[cos(Theta3),sin(Theta3),0];drop3=FY3+d3*v3*t3;
    C1=[drop1(1)+v1*d1(1)*delay1,drop1(2)+v1*d1(2)*delay1,drop1(3)-0.5*g*delay1^2];
    C2=[drop2(1)+v2*d2(1)*delay2,drop2(2)+v2*d2(2)*delay2,drop2(3)-0.5*g*delay2^2];
    C3=[drop3(1)+v3*d3(1)*delay3,drop3(2)+v3*d3(2)*delay3,drop3(3)-0.5*g*delay3^2];
    if C1(3)<0,C1(3)=0;end
    if C2(3)<0,C2(3)=0;end
    if C3(3)<0,C3(3)=0;end
    
    t_explosion=[t_explosion1,t_ex2,t_ex3];
    explosion_position=[C1;C2;C3];
    
    t_start=max(min(t_explosion),0);
    t_end=min(max(t_explosion)+c_cover,t_boom);
    if t_end<=t_start
        cover_time=0;return;
    end
    
    %主循环
    cover_time=0;
    uM=(fake_target-M1)/norm(fake_target-M1);
    t=t_start;
    while t<=t_end+1e-12
        M_t=M1+uM*V_missile*t;
        covered=false;
        for j=1:3
            t_explosionj=t_explosion(j);
            if t<t_explosionj||t>t_explosionj+c_cover,continue;end
            C_t=explosion_position(j,:)+[0,0,-c_down*(t-t_explosionj)];
            v=true_target-M_t;
            vv=dot(v,v);
            if vv==0
                s_clamped=0;
            else
                s=dot(C_t-M_t,v)/vv;
                s_clamped=max(0,min(1,s));
            end
            closest=M_t+s_clamped*v;
            dist=norm(C_t-closest);
            if dist<=R
                covered=true;
                break;
            end
        end
        if covered
            cover_time=cover_time+dt;
        end
        t=t+dt;
    end
end

%% 计算总体遮蔽区间函数
function [cover_time,intervals]=cover_intervals(x,M1,V_missile,fake_target,true_target,...
    FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom)

    Theta1=mod(x(1),2*pi);v1=x(2);t1=x(3);delay1=x(4);
    Theta2=mod(x(5),2*pi);v2=x(6);t2=x(7);delay2=x(8);
    Theta3=mod(x(9),2*pi);v3=x(10);t3=x(11);delay3=x(12);
    t_explosion1=t1+delay1;t_ex2=t2+delay2;t_ex3=t3+delay3;
    
    d1=[cos(Theta1),sin(Theta1),0];drop1=FY1+d1*v1*t1;
    d2=[cos(Theta2),sin(Theta2),0];drop2=FY2+d2*v2*t2;
    d3=[cos(Theta3),sin(Theta3),0];drop3=FY3+d3*v3*t3;
    C1=[drop1(1)+v1*d1(1)*delay1,drop1(2)+v1*d1(2)*delay1,drop1(3)-0.5*g*delay1^2];
    C2=[drop2(1)+v2*d2(1)*delay2,drop2(2)+v2*d2(2)*delay2,drop2(3)-0.5*g*delay2^2];
    C3=[drop3(1)+v3*d3(1)*delay3,drop3(2)+v3*d3(2)*delay3,drop3(3)-0.5*g*delay3^2];
    if C1(3)<0,C1(3)=0;end
    if C2(3)<0,C2(3)=0;end
    if C3(3)<0,C3(3)=0;end
    
    t_explosion=[t_explosion1,t_ex2,t_ex3];
    expl_pos=[C1;C2;C3];
    t_start=max(min(t_explosion),0);
    t_end=min(max(t_explosion)+c_cover,t_boom);
    
    cover_time=0;intervals=[];
    if t_end<=t_start,return;end
    
    uM=(fake_target-M1)/norm(fake_target-M1);
    shielding=false;cur_start=NaN;
    t=t_start;
    while t<=t_end+1e-12
        M_t=M1+uM*V_missile*t;
        covered=false;
        for j=1:3
            t_exj=t_explosion(j);
            if t<t_exj||t>t_exj+c_cover,continue;end
            C_t=expl_pos(j,:)+[0,0,-c_down*(t-t_exj)];
            v=true_target-M_t;
            vv=dot(v,v);
            if vv==0
                s_clamped=0;
            else
                s=dot(C_t-M_t,v)/vv;
                s_clamped=max(0,min(1,s));
            end
            closest=M_t+s_clamped*v;
            d=norm(C_t-closest);
            if d<=R
                covered=true;
                break;
            end
        end
        if covered
            cover_time=cover_time+dt;
            if ~shielding
                shielding=true;cur_start=t;
            end
        else
            if shielding
                shielding=false;
                intervals=[intervals;cur_start,t-dt];
                cur_start=NaN;
            end
        end
        t=t+dt;
    end
    if shielding
        intervals=[intervals;cur_start,t_end];
    end
end

%% 计算单枚弹的遮蔽区间
function [cover_time,intervals]=single_bomb_interval(t_explosion_i,explosion_position_i,idx_bomb,x,...
    M1,V_missile,fake_target,true_target,FY1,FY2,FY3,g,R,c_down,c_cover,dt,t_boom)

    Theta1=mod(x(1),2*pi);v1=x(2);t1=x(3);delay1=x(4);
    Theta2=mod(x(5),2*pi);v2=x(6);t2=x(7);delay2=x(8);
    Theta3=mod(x(9),2*pi);v3=x(10);t3=x(11);delay3=x(12);
    
    %当前单枚干扰弹有效存在区间
    if t_explosion_i>t_boom
        cover_time=0;intervals=[];return;
    end
    t_start=max(t_explosion_i,0);
    t_end=min(t_explosion_i+c_cover,t_boom);
    if t_end<=t_start
        cover_time=0;intervals=[];return;
    end
    %判断该干扰弹是否在该时刻遮蔽连线
    cover_time=0;intervals=[];
    shielding=false;cur_start=NaN;
    uM=(fake_target-M1)/norm(fake_target-M1);
    t=t_start;
    while t<=t_end+1e-12
        M_t=M1+uM*V_missile*t;
        %时间t时烟幕球心
        C_t=explosion_position_i+[0,0,-c_down*(t-t_explosion_i)];
        v=true_target-M_t;
        vv=dot(v,v);
        if vv==0
            s_clamped=0;
        else
            s=dot(C_t-M_t,v)/vv;
            s_clamped=max(0,min(1,s));
        end
        closest=M_t+s_clamped*v;
        dist=norm(C_t-closest);
        if dist<=R
            cover_time=cover_time+dt;
            if ~shielding
                shielding=true;cur_start=t;
            end
        else
            if shielding
                shielding=false;
                intervals=[intervals;cur_start,t-dt];
                cur_start=NaN;
            end
        end
        t=t+dt;
    end
    if shielding
        intervals=[intervals;cur_start,t_end];
    end
end
