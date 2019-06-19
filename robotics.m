%% Project���˹������������2018 ���е��������Ŀ
%  Author��������(2016141067)
%  Date��2018-12-13
%  Email��16tlluo@stu.edu.cn

%% ����Robotic Toolbox ����3���ɶȻ�е��
clf;clc;clear;close all;

% ����DH�����������˶���
L(1) = Link([0 0 0.5 -pi/2 0]);
L(2) = Link([0 0 1.0 pi/2 0]);
L(3) = Link([0 0 1.0 0 0]);
% ���������˻����ˣ�threelink
threelink = SerialLink(L, 'name', 'threelink');
% ����������ͼ��(�������Ϊ�ؽڱ���)
syms qn;
qz = [0 0 0];       % ��ʼλ�ùؽڱ���
qn = [0 pi/4 0];    % ����λ��1
% threelink.fkine(qn)
threelink.plot(qn)  % ����е���ƶ�������λ��1
threelink.display   % �����е����Ϣ
%threelink.teach    

%% ��е�ۿ�����
input_flag = 0;     % ������ɱ�־
quit = 0;           % �˳���־
while(quit == 0)
    %% �������ֱ꣬��������������
    % ����ĩ������[dx dy dz]�����ڻ�������ϵ
    while(input_flag == 0)
        fprintf('������Ŀ������(���ڻ�������ϵ)��\n');
        dx = input('Please input dx:');
        dy = input('Please input dy:');
        dz = input('Please input dz:');
        dest = [dx;dy;dz];    
        % ��Ŀ������ת��Ϊ��α任����
        T = transl(dest);
        % ���˶�ѧ���ؽڽǶȣ������⣩
        dq = threelink.ikine(T,'mask',[1 1 1 0 0 0]);
        if (length(dq) ==  3)
            fprintf('���˶�ѧ�����ɣ���ʼ�ƶ�������......\n');
            input_flag = 1;
            break
        else
            fprintf('Ŀ�����겻���㣬������ѡ��Ŀ�����꣡\n');
        end
    end
    
    %% ���˶�ѧ�����ɣ�׼���ƶ�������
    % ����ʱ����
    t = [0:0.05:2]';
    % ��ȡ��ǰλ�ιؽڽǶ�
    qn = threelink.getpos;
    % ͨ����ֵ�γɿռ�켣qn->dq
    q_traj = jtraj(qn, dq, t);
    % ���ƶ���
    figure(1)
    threelink.plot(q_traj);
    % �˶����̷���
    figure(2)
    subplot(1,1,1); plot(t,q_traj);
    xlabel('t'); ylabel('�ؽڽǶ�');
    legend('q1', 'q2', 'q3');
    title('�ؽڽǶȱ仯����');
    % ��ȡ��ǰ�������Ӧ�ĹؽڽǶ�
    current_pos = threelink.getpos;
    % ���õ�ǰ�����Ӧ�ĹؽڽǶ�Ϊqn
    qn = current_pos;
    % �������
    error = sum(abs(dq - current_pos));
    
    %% �����������
    if(error < 0.02)
        fprintf('==== �������ƶ���ɣ�====\n')
        quit = input('�˳����������ƶ�������(1/0)?:');
        if (quit)
            break               % �˳�����
        else
            input_flag = 0;     % �������У���������Ŀ��
        end        
    end
end
close all;                      % �˳�ʱ�ر����д���
