clear; clc; rng(0);

%% 常量参数
% 各点坐标
M1=[20000,0,2000];    
V_missile=300;    
fake_target=[0,0,0];
true_target=[0,200,5];    %取真目标中心高度5
FY1=[17800,0,1800];
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
t1_min=0;       %投放时间范围
t1_max=t_boom;
interval_min=1; interval_max=min(floor(t_boom/2),30);
delay_min=0;delay_max=c_cover;        % 起爆延时不大于烟幕球存在时间

%% 差分进化参数
NP=60;        %种群规模大小
Gmax=500;     %最多迭到第几代
F=0.8;        %缩放因子大小
CR=0.9;       %交叉的概率

%% 初始化种群矩阵
D=7;
po=zeros(NP,D);
for i=1:NP       
    po(i,1)=Theta_min+(Theta_max-Theta_min)*rand();              % 航向角theta
    po(i,2)=v_min+(v_max-v_min)*rand();                          % 无人机速度v
    po(i,3)=t1_min+(t1_max-t1_min)*rand();                       % 投弹时间t1
    po(i,4)=interval_min+(interval_max-interval_min)*rand();     % 间隔Delta
    po(i,5)=delay_min+(delay_max-delay_min)*rand();              % 干扰弹1延时引爆时间
    po(i,6)=delay_min+(delay_max-delay_min)*rand();              % 干扰弹2延时引爆时间
    po(i,7)=delay_min+(delay_max-delay_min)*rand();              % 干扰弹3延时引爆时间
end

% 评估初代适应度
fitness = zeros(NP,1);
for i=1:NP
    fitness(i)=fitness_cover(po(i,:),M1,V_missile,fake_target,true_target,FY1,...
       g,R,c_down,c_cover,dt,t_boom,Theta_min,Theta_max,v_min,v_max,t1_min,t1_max,interval_min,interval_max,delay_min,delay_max);
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
        jrand=randi(D);
        for j = 1:D
            if rand<=CR||j==jrand
                u(j)=v_vec(j);
            end
        end

        u(1)=mod(u(1),2*pi);            %处理边界
        u(2)=max(v_min,min(v_max,u(2)));
        u(3)=max(t1_min,min(t1_max,u(3)));
        u(4)=max(interval_min,min(interval_max,u(4)));
        u(5)=max(delay_min,min(delay_max,u(5)));
        u(6)=max(delay_min,min(delay_max,u(6)));
        u(7)=max(delay_min,min(delay_max,u(7)));
        
        % 计算适应度
        f=fitness_cover(u,M1,V_missile,fake_target,true_target,FY1,...
             g,R,c_down,c_cover,dt,t_boom,Theta_min,Theta_max,v_min,v_max,t1_min,t1_max,interval_min,interval_max,delay_min,delay_max);
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
fprintf('\nProblem3\n');
fprintf('最大遮蔽时长为%.2fs\n', best_f);
Theta = best_x(1); v_best = best_x(2); t1 = best_x(3); interval = best_x(4);
delay1 = best_x(5); delay2 = best_x(6); delay3 = best_x(7);
fprintf('最优参数：航向角为%.3f(deg),无人机速度为%.3fm/s,投放时间为%.2fs,投放间隔%.2fs\n第一枚干扰弹的爆炸延时为%.3fs\n第二枚干扰弹的爆炸延时为%.3fs\n第三枚干扰弹的爆炸延时为%.3fs\n', ...
    Theta*180/pi,v_best,t1,interval,delay1,delay2,delay3);

% 计算并打印投放点和起爆点以及总遮蔽区间
t2=t1+interval;t3=t1+2*interval;
t_ex1=t1+delay1;t_ex2=t2+delay2;t_ex3=t3+delay3;
d_u=[cos(Theta),sin(Theta),0];
drop1=FY1+d_u*v_best*t1;
drop2=FY1+d_u*v_best*t2;
drop3=FY1+d_u*v_best*t3;
explosion1=[drop1(1)+v_best*d_u(1)*delay1,drop1(2)+v_best*d_u(2)*delay1,drop1(3)-0.5*g*delay1^2];
explosion2=[drop2(1)+v_best*d_u(1)*delay2,drop2(2)+v_best*d_u(2)*delay2,drop2(3)-0.5*g*delay2^2];
explosion3=[drop3(1)+v_best*d_u(1)*delay3,drop3(2)+v_best*d_u(2)*delay3,drop3(3)-0.5*g*delay3^2];

fprintf('干扰弹1投放坐标(%.3f,%.3f,%.3f),投放时间为%.2fs,爆炸时间为=%.2fs,爆炸坐标为(%.3f,%.3f,%.3f)\n',...
    drop1,t1,t_ex1,explosion1);
fprintf('干扰弹2投放坐标(%.3f,%.3f,%.3f),投放时间为%.2fs,爆炸时间为=%.2fs,爆炸坐标为(%.3f,%.3f,%.3f)\n',...
    drop2,t2,t_ex2,explosion2);
fprintf('干扰弹3投放坐标(%.3f,%.3f,%.3f),投放时间为%.2fs,爆炸时间为=%.2fs,爆炸坐标为(%.3f,%.3f,%.3f)\n',...
    drop3,t3,t_ex3,explosion3);

% 计算总遮蔽区间
[cover_time,intervals]=cover_intervals([Theta,v_best,t1,interval,delay1,delay2,delay3],...
                                       M1,V_missile,fake_target,true_target,FY1,g,R,c_down,c_cover,dt,t_boom);
if isempty(intervals)
    fprintf('未检测到任何总体遮蔽区间，即覆盖时长为 0\n');
else
    fprintf('检测到%d个遮蔽区间：\n', size(intervals,1));
    for k=1:size(intervals,1)
        fprintf('区间%d: %.2fs~%.2fs (时长=%.2fs)\n',k,intervals(k,1),intervals(k,2),intervals(k,2)-intervals(k,1)+dt);
    end
end

%% 为每枚干扰弹计算有效遮蔽时间区间
for i=1:3
    switch i
        case 1
            tex=t_ex1;position=explosion1;
        case 2
            tex=t_ex2;position=explosion2;
        case 3
            tex=t_ex3;position=explosion3;
    end
    [ct_i,intervals_i]=single_bomb_interval(tex,position,i,[Theta,v_best,t1,interval,delay1,delay2,delay3],...
                                                     M1,V_missile,fake_target,true_target,FY1,g,R,c_down,c_cover,dt,t_boom);
    fprintf('干扰弹%d单独遮蔽时长为%.2fs\n',i,ct_i);
    if isempty(intervals_i)
        fprintf('干扰弹%d未产生遮蔽时段区间。\n',i);
    else
        fprintf('干扰弹%d遮蔽区间为：\n',i);
        for kk=1:size(intervals_i,1)
            fprintf('    %.2fs~%.2fs,时长为%.2fs\n',intervals_i(kk,1),intervals_i(kk,2),intervals_i(kk,2)-intervals_i(kk,1)+dt);
        end
    end
end

%% 目标函数
function cover_time=fitness_cover(x,M1,V_missile,fake_target,true_target,FY1,...
    g,R,c_down,c_cover,dt,t_boom,theta_min,theta_max,v_min,v_max,t1_min,t1_max,interval_min,interval_max,delay_min,delay_max)

    Theta=mod(x(1),2*pi);
    v_u=max(v_min,min(v_max,x(2)));
    t1=max(t1_min,min(t1_max,x(3)));
    interval=max(interval_min,min(interval_max,x(4)));
    delay1=max(delay_min,min(delay_max,x(5)));
    delay2=max(delay_min,min(delay_max,x(6)));
    delay3=max(delay_min,min(delay_max,x(7)));
    
    t2=t1+interval;t3=t1+2*interval;
    
    if (t1+delay1)>t_boom&&(t2+delay2)>t_boom&&(t3+delay3)>t_boom   %排除起爆时间晚于导弹到达时间
       cover_time=0;
        return;
    end
    
    d_unit=[cos(Theta),sin(Theta),0];
    % 计算投放点与爆炸点
    drop_po1=FY1+d_unit*v_u*t1;
    drop_po2=FY1+d_unit*v_u*t2;
    drop_po3=FY1+d_unit*v_u*t3;
    C1=[drop_po1(1)+v_u*d_unit(1)*delay1,drop_po1(2)+v_u*d_unit(2)*delay1,drop_po1(3)-0.5*g*delay1^2];
    C2=[drop_po2(1)+v_u*d_unit(1)*delay2,drop_po2(2)+v_u*d_unit(2)*delay2,drop_po2(3)-0.5*g*delay2^2];
    C3=[drop_po3(1)+v_u*d_unit(1)*delay3,drop_po3(2)+v_u*d_unit(2)*delay3,drop_po3(3)-0.5*g*delay3^2];
    
    % 若某爆炸点z<0,设为z=0
    if C1(3)<0,C1(3)=0;end
    if C2(3)<0,C2(3)=0;end
    if C3(3)<0,C3(3)=0;end
    
    t_explosion=[t1+delay1,t2+delay2,t3+delay3];
    expl_pos=[C1;C2;C3];

    t_start=max(min(t_explosion),0);
    t_end=min(max(t_explosion)+c_cover,t_boom);
    if t_end<=t_start
        cover_time=0;
        return;
    end
    
    % 主循环
    cover_time=0;
    uM=(fake_target-M1)/norm(fake_target-M1);
    t=t_start;
    while t<=t_end+1e-12
        M_t=M1+uM*V_missile*t;
        covered=false;
        for i=1:3
            t_exi=t_explosion(i);
            if t<t_exi||t>t_exi+c_cover
                continue;
            end
            C_t=expl_pos(i,:)+[0,0,-c_down*(t-t_exi)];
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
function [cover_time,intervals]=cover_intervals(x,M0,V_missile,fake_target,true_target,FY1, ...
                                g,R,c_down,c_cover,dt,t_boom)
    
    Theta=mod(x(1),2*pi);
    v_u=x(2);
    t1=x(3);
    interval=x(4);
    delay1=x(5);delay2=x(6);delay3=x(7);
    t2=t1+interval;t3=t1+2*interval;
    
    d_unit=[cos(Theta),sin(Theta),0];
    drop1=FY1+d_unit*v_u*t1;
    drop2=FY1+d_unit*v_u*t2;
    drop3=FY1+d_unit*v_u*t3;
    C1=[drop1(1)+v_u*d_unit(1)*delay1,drop1(2)+v_u*d_unit(2)*delay1,drop1(3)-0.5*g*delay1^2];
    C2=[drop2(1)+v_u*d_unit(1)*delay2,drop2(2)+v_u*d_unit(2)*delay2,drop2(3)-0.5*g*delay2^2];
    C3=[drop3(1)+v_u*d_unit(1)*delay3,drop3(2)+v_u*d_unit(2)*delay3,drop3(3)-0.5*g*delay3^2];
    if C1(3)<0,C1(3)=0;end
    if C2(3)<0,C2(3)=0;end
    if C3(3)<0,C3(3)=0;end
    t_explosion=[t1+delay1,t2+delay2,t3+delay3];
    explosion_position=[C1;C2;C3];
    
    t_start=max(min(t_explosion),0);
    t_end=min(max(t_explosion)+c_cover,t_boom);
    cover_time=0;intervals=[];if t_end<=t_start,return;end
    
    uM=(fake_target-M0)/norm(fake_target-M0);
    shielding=false;cur_start=NaN;
    t=t_start;
    while t<=t_end+1e-12
        M_t=M0+uM*V_missile*t;
        covered=false;
        for i=1:3
            t_exi=t_explosion(i);
            if t<t_exi||t>t_exi+c_cover,continue;end
            C_t=explosion_position(i,:)+[0,0,-c_down*(t-t_exi)];
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
                covered=true;break;
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
                intervals=[intervals;cur_start,t-dt];%#ok<AGROW>
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
    M1,V_missile,fake_target,true_target,FY1,g,R,c_down,c_cover,dt,t_boom)
    Theta=mod(x(1),2*pi);
    v_u=x(2);
    t1=x(3);
    interval=x(4);
    delay=x(5);delay2=x(6);delay3=x(7);
    t2=t1+interval;t3=t1+2*interval;
    t_expl=[t1+delay,t2+delay2,t3+delay3];
    
    %当前干扰弹的有效存在区间
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
    shielding=false;cur_s=NaN;
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
                shielding=true;cur_s=t;
            end
        else
            if shielding
                shielding=false;
                intervals=[intervals;cur_s,t-dt];
                cur_s=NaN;
            end
        end
        t=t+dt;
    end
    if shielding
        intervals=[intervals;cur_s,t_end];
    end
end