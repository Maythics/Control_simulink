clc
m = 0.035;  % 左边为原始数据，这是在模糊+神经网络中更改的：0.05;0.3;
r = 0.0672/2;
I = 0.5*m*r^2;
M = 0.757-2*m;
l = 0.5*0.0903; % 左边为原始数据，这是在模糊+神经网络中更改的：0.5*0.2; 0.5*0.4;
J_p = (1/12)*M*(0.0903^2+0.0530^2);
d = 0.1612;
J_delta = (1/12)*M*(0.0930^2+0.0530^2);
g = 9.8;


Ts = 0.1; % 模糊控制器的采样时间

% 神经网络系数范围确定
Kp = -1000; % 使得P处于[-1000,0]的范围中
Ki = -1000; % 使得I处于[-1000,0]的范围中
Kd = -2000; % 使得D处于[-1000,0]的范围中

T = 0.02; % 神经网络pid控制器的采样时间
alpha = 0.05; % 学习率