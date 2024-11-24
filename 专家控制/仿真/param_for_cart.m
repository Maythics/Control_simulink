%% 离散PID参数 
m = 0.5;
M = 1;
l = 0.5;
g = 9.8;
T = 0.0001; % sample period
K_p = 200;
T_i = 0.001;
T_d = 10;
F_m =25; % max input
K = 1;

%% 专家控制部分参数

theta_1 = 0.1;
theta_2 = 0.3;
theta_m = 0.5;
K_s = 1;
K_b = 1.3;
