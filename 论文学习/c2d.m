% 定义连续系统
A_cont = [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
];
B_cont = [0 0 0;
    0 0 0;
    0 0 0;
    1 0 0;
    0 1 0;
    0 0 1;
    ];        % 连续输入矩阵B
C_cont = eye(6);        % 输出矩阵（可根据实际情况定义）
D_cont = zeros(6,3);    % 直接传输矩阵（可根据实际情况定义）

% 创建连续状态空间模型
sys_cont = ss(A_cont, B_cont, C_cont, D_cont);

% 定义采样周期T（单位：秒）
T = 0.1;

% 使用0阶保持器（ZOH）进行离散化
sys_disc = c2d(sys_cont, T, 'zoh');  % 'zoh'是默认值，可省略

% 提取离散化后的A和B矩阵
A_disc = sys_disc.A;  % 离散状态矩阵A_d
B_disc = sys_disc.B;  % 离散输入矩阵B_d

% 显示结果
disp('离散化后的A矩阵：');
disp(A_disc);
disp('离散化后的B矩阵：');
disp(B_disc);
