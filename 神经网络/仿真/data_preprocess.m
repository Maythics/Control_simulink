% 转置，得到行向量
q1 = out.q1';
q2 = out.q2';
% 使用向前平移一位的方法，构造q1(k-1)以及q2(k-2)
q1_shift = [0,q1(1:end-1)];
q2_shift = [0,q2(1:end-1)];
tau1 = out.tau1';
tau2 = out.tau2';

% 拼接成一个用于训练的输入数据
data_combine = [q1;q2;q1_shift;q2_shift;tau1;tau2]; 
% 去掉最后一列是为了和label对齐，因为最后一个时刻的label在未来，得不到
input_data = data_combine(:,1:end-1);

%ground truth是下一个时刻的q1和q2两个角度，因此是后移一位：
q1_next = q1(2:end);
q2_next = q2(2:end);
% 拼接成一个用于训练的标签数据
label_data = [q1_next;q2_next];