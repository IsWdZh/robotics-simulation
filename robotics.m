%% Project：人工智能与机器人2018 秋机械臂三级项目
%  Author：罗天林(2016141067)
%  Date：2018-12-13
%  Email：16tlluo@stu.edu.cn

%% 基于Robotic Toolbox 构建3自由度机械臂
clf;clc;clear;close all;

% 根据DH参数创建连杆对象
L(1) = Link([0 0 0.5 -pi/2 0]);
L(2) = Link([0 0 1.0 pi/2 0]);
L(3) = Link([0 0 1.0 0 0]);
% 构造三连杆机器人：threelink
threelink = SerialLink(L, 'name', 'threelink');
% 画出机器人图形(传入参数为关节变量)
syms qn;
qz = [0 0 0];       % 初始位置关节变量
qn = [0 pi/4 0];    % 测试位置1
% threelink.fkine(qn)
threelink.plot(qn)  % 将机械臂移动到测试位置1
threelink.display   % 输出机械臂信息
%threelink.teach    

%% 机械臂控制器
input_flag = 0;     % 输入完成标志
quit = 0;           % 退出标志
while(quit == 0)
    %% 输入坐标，直到坐标满足条件
    % 输入末端坐标[dx dy dz]，基于基座坐标系
    while(input_flag == 0)
        fprintf('请输入目标坐标(基于基座坐标系)：\n');
        dx = input('Please input dx:');
        dy = input('Please input dy:');
        dz = input('Please input dz:');
        dest = [dx;dy;dz];    
        % 将目标坐标转换为齐次变换矩阵
        T = transl(dest);
        % 逆运动学求解关节角度（迭代解）
        dq = threelink.ikine(T,'mask',[1 1 1 0 0 0]);
        if (length(dq) ==  3)
            fprintf('逆运动学求解完成，开始移动机器人......\n');
            input_flag = 1;
            break
        else
            fprintf('目标坐标不满足，请重新选择目标坐标！\n');
        end
    end
    
    %% 逆运动学求解完成，准备移动机器人
    % 设置时间间隔
    t = [0:0.05:2]';
    % 获取当前位形关节角度
    qn = threelink.getpos;
    % 通过插值形成空间轨迹qn->dq
    q_traj = jtraj(qn, dq, t);
    % 绘制动画
    figure(1)
    threelink.plot(q_traj);
    % 运动过程分析
    figure(2)
    subplot(1,1,1); plot(t,q_traj);
    xlabel('t'); ylabel('关节角度');
    legend('q1', 'q2', 'q3');
    title('关节角度变化曲线');
    % 获取当前的坐标对应的关节角度
    current_pos = threelink.getpos;
    % 设置当前坐标对应的关节角度为qn
    qn = current_pos;
    % 计算误差
    error = sum(abs(dq - current_pos));
    
    %% 任务完成条件
    if(error < 0.02)
        fprintf('==== 机器人移动完成！====\n')
        quit = input('退出程序或继续移动机器人(1/0)?:');
        if (quit)
            break               % 退出程序
        else
            input_flag = 0;     % 继续运行，重新输入目标
        end        
    end
end
close all;                      % 退出时关闭所有窗口
