rng(0); %随机种子

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
t_drop_min=0;       %投放时间范围
t_drop_max=t_boom;
delay_min=0;        %起爆延时范围
delay_max=t_boom; 

%% 差分进化参数
NP=100;       %种群规模大小
Gmax=500;     %最多迭到第几代
F=0.8;        %缩放因子大小
CR=0.9;       %交叉的概率

%% 初始化种群矩阵
D=4;
po=zeros(NP,D);
for i=1:NP
    po(i,1)=Theta_min+(Theta_max-Theta_min)*rand();     % 航向角theta
    po(i,2)=v_min+(v_max-v_min)*rand();                 % 无人机速度v
    po(i,3)=t_drop_min+(t_drop_max-t_drop_min)*rand();  % 投弹时间t_drop
    po(i,4)=delay_min+(delay_max-delay_min)*rand();     % 延时爆炸delay
end

% 评估初代适应度
fitness=zeros(NP,1);
for i=1:NP
    fitness(i)=fitness_cover(po(i,:),M1,V_missile,fake_target,true_target,...
    FY1,g,R,c_down,c_cover,dt,t_boom);
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
        
        u(1)=max(Theta_min, min(Theta_max, u(1)));  % 处理边界
        u(2)=max(v_min, min(v_max, u(2)));
        u(3)=max(t_drop_min, min(t_drop_max, u(3)));
        u(4)=max(delay_min, min(delay_max, u(4)));
        
        f=fitness_cover(u,M1,V_missile,fake_target,true_target,...%计算候选解的适应度
                        FY1,g,R,c_down,c_cover,dt,t_boom);
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
fprintf('\nProblem2\n');
fprintf('最大遮蔽时长为%.2fs\n', best_f);
fprintf('最优参数：航向角为%.3f(rad),无人机速度为%.3fm/s,投放时间%.3fs,延时起爆时间为%.3fs)\n', ...
    best_x(1)*180/pi, best_x(2), best_x(3), best_x(4));

% 打印起爆时间与起爆位置
t_explosion=best_x(3)+best_x(4);
d_unit=[cos(best_x(1)),sin(best_x(1)),0];%计算爆炸位置
drop_position=FY1+d_unit*best_x(2)*best_x(3);
explosion_position=[drop_position(1)+best_x(2)*d_unit(1)*best_x(4),...
                    drop_position(2)+best_x(2)*d_unit(2)*best_x(4),...
                    drop_position(3)-0.5*g*best_x(4)^2];
fprintf('最佳起爆时间%.3fs\n',t_explosion);
fprintf('最佳起爆位置为(%.3f,%.3f,%.3f)\n',explosion_position);


%% 累计遮蔽时间计算函数
function cover_time = fitness_cover(x,M1,V_missile,fake_target,true_target,...
    FY1,g,R,c_down,c_cover,dt,t_boom)

    theta=x(1);
    v_u=x(2);
    t_drop=x(3);
    delay=x(4);
    
    d_unit=[cos(theta),sin(theta),0];       %无人机方向
    drop_pos=FY1+d_unit*v_u*t_drop;             % 计算投放点
    v_b=v_u*d_unit;                          % 弹体初速度
    t_explosion=t_drop+delay;                   % 起爆时间与位置
    x_bomb=drop_pos(1)+v_b(1)*delay;
    y_bomb=drop_pos(2)+v_b(2)*delay;
    z_bomb=drop_pos(3)+v_b(3)*delay-0.5*g*delay^2;
    C0=[x_bomb,y_bomb,z_bomb];
    
    if t_explosion>t_boom       % 排除起爆时间晚于导弹到达时间
        cover_time=0;
        return;
    end
    
    t_start=t_explosion;
    t_end=min(t_explosion+c_cover,t_boom);
    if t_end<=t_start
        cover_time=0;
        return;
    end
    
    cover_time=0; % 逐步检测
    uM=(fake_target-M1)/norm(fake_target-M1);
    
    t = t_start;
    while t<=t_end+1e-12
        M_t=M1+uM*V_missile*t;                %导弹在全局时间t的位置
        C_t=C0+[0,0,-c_down*(t-t_explosion)]; %烟幕球心位置
        % 判断连线导弹-烟幕球与烟幕球是否相交
        v=true_target-M_t;
        vv=dot(v,v);
        if vv==0
           s_clamped=0;
        else
           w=C_t-M_t;
           s=dot(w,v)/vv;
           s_clamped=max(0,min(1,s));
        end
           closest=M_t+s_clamped*v;
           dist=norm(C_t-closest);
        if dist<=R
           cover_time=cover_time+dt;
        end
        t=t+dt;
    end
end
