%常量与初始值
M1=[20000,0,2000];%导弹M1初始位置
fake_target=[0,0,0];%假目标/原点
true_target=[0,200,0];%真目标位置
V_m=300;%导弹速度

%计算导弹单位方向向量 导弹-假目标
u=(fake_target-M1);
u=u/norm(u);

t_explosion=5.1;%起爆时间
c0=[17188,0,1736.496];%起爆点
c_down=3;%烟幕球以3m/s向下匀速下沉
R=10;%有效遮蔽半径
c_cover=20;

dt=0.01;%设置步长
t_start=t_explosion;
t_end=t_explosion+c_cover;
t_vec=t_start:dt:t_end;

%初始化计数器与存储
total_time=0;
shielding=false;
interval=[];%将存储遮蔽的开始时间和结束时间
current_start=NaN;

%按时间步检测连线与球是否相交
for k=1:numel(t_vec)
    t=t_vec(k);
    M_t=M1+u*V_m*t;
    T=true_target;
    C_t=c0+[0,0,-c_down*(t-t_explosion)];%烟幕球随时间下沉
    
    %计算导弹-目标线段与球的最短距离
    v=T-M_t;%线段方向向量
    w=C_t-M_t;%从导弹位置到球心向量
    vv=dot(v,v);
    if vv==0  % 若导弹与目标重合
        s=0;
    else
        s=dot(w,v)/vv;  % 投影参数
    end
    % 将投影参数限制到线段 [0,1]
    s_clamped=max(0,min(1,s));
    closest=M_t+s_clamped*v;
    dist=norm(C_t-closest);
    
    if dist<=R  % 判断是否相交
        total_time= total_time+dt;
        if ~shielding  % 则新遮蔽区间开始
            
            shielding=true;
            current_start=t;
        end
    else
        if shielding
            % 遮蔽区间结束并保存
            shielding=false;
            interval=[interval;current_start,t-dt];
            current_start=NaN;
        end
    end
end

% 若在循环结束时仍处于遮蔽状态，补最后一个区间
if shielding
    interval=[interval;current_start,t_end];
end

%输出结果
fprintf('检测时间段%.2fs 到%.2fs（步长 %.2fs）\n',t_start,t_end,dt);
fprintf('累计遮蔽时长=%.2fs\n',total_time);
if isempty(interval)
    fprintf('没有检测到任何遮蔽区间。\n');
else
    fprintf('检测到%d个遮蔽区间：\n', size(interval,1));
    for i=1:size(interval,1)
        fprintf('区间%d:从%.2fs到%.2fs，时长=%.2fs\n',...
            i, interval(i,1), interval(i,2), interval(i,2)-interval(i,1)+dt);
    end
end