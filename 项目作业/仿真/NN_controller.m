function NN_controller(block)

  setup(block);
  
%end function

function setup(block)
  
  block.NumDialogPrms  = 0;
  
  %% Register number of input and output ports
  block.NumInputPorts  = 3; % e(k) e(k-1) e(k-2) e(k-3)
  block.NumOutputPorts = 1; % 输出的是PID向量

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 4;
  block.InputPort(1).DirectFeedthrough = false;

  block.InputPort(2).Dimensions        = 2; % y(k)与y(k-1)
  block.InputPort(2).DirectFeedthrough = false;

  block.InputPort(3).Dimensions        = 2; % u(k)与u(k-1)
  block.InputPort(3).DirectFeedthrough = false;

  block.OutputPort(1).Dimensions       = 3;
  
  %% Set block sample time to [0.0001 0]
  T = evalin('base', 'T');
  block.SampleTimes = [T 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);    
  block.RegBlockMethod('Update',                  @Update);  
  block.RegBlockMethod('Outputs',                 @Output);
  
%endfunction

function DoPostPropSetup(block)

  %% Setup Dwork
  block.NumDworks = 10;
  
  % 一个内置更新量为W1
  block.Dwork(1).Name = 'W1'; 
  block.Dwork(1).Dimensions      = 4;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

  % 一个内置更新量为W2
  block.Dwork(2).Name = 'W2'; 
  block.Dwork(2).Dimensions      = 4;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;

  % 一个内置更新量为W3
  block.Dwork(3).Name = 'W3'; 
  block.Dwork(3).Dimensions      = 4;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;

  % 一个内置更新量为net
  block.Dwork(4).Name = 'net'; 
  block.Dwork(4).Dimensions      = 3;
  block.Dwork(4).DatatypeID      = 0;
  block.Dwork(4).Complexity      = 'Real';
  block.Dwork(4).UsedAsDiscState = true;

  % 一个内置更新量为hidden_output(即sigmoid(net))
  block.Dwork(5).Name = 'hidden_output'; 
  block.Dwork(5).Dimensions      = 3;
  block.Dwork(5).DatatypeID      = 0;
  block.Dwork(5).Complexity      = 'Real';
  block.Dwork(5).UsedAsDiscState = true;

  % 一个内置更新量为K1（输出层权重）
  block.Dwork(6).Name = 'K1'; 
  block.Dwork(6).Dimensions      = 3;
  block.Dwork(6).DatatypeID      = 0;
  block.Dwork(6).Complexity      = 'Real';
  block.Dwork(6).UsedAsDiscState = true;

  % 一个内置更新量为K2（输出层权重）
  block.Dwork(7).Name = 'K2'; 
  block.Dwork(7).Dimensions      = 3;
  block.Dwork(7).DatatypeID      = 0;
  block.Dwork(7).Complexity      = 'Real';
  block.Dwork(7).UsedAsDiscState = true;

  % 一个内置更新量为K3（输出层权重）
  block.Dwork(8).Name = 'K3'; 
  block.Dwork(8).Dimensions      = 3;
  block.Dwork(8).DatatypeID      = 0;
  block.Dwork(8).Complexity      = 'Real';
  block.Dwork(8).UsedAsDiscState = true;

  % 一个内置更新量为SUM
  block.Dwork(9).Name = 'SUM'; 
  block.Dwork(9).Dimensions      = 3;
  block.Dwork(9).DatatypeID      = 0;
  block.Dwork(9).Complexity      = 'Real';
  block.Dwork(9).UsedAsDiscState = true;

  % 一个内置更新量为output(即sigmoid(SUM))
  block.Dwork(10).Name = 'output'; 
  block.Dwork(10).Dimensions      = 3;
  block.Dwork(10).DatatypeID      = 0;
  block.Dwork(10).Complexity      = 'Real';
  block.Dwork(10).UsedAsDiscState = true;


%endfunction

function InitConditions(block)

  %% Initialize Dwork
  block.Dwork(1).Data = rand(4,1);
  block.Dwork(2).Data = rand(4,1);
  block.Dwork(3).Data = rand(4,1);
  block.Dwork(4).Data = rand(3,1);
  block.Dwork(5).Data = rand(3,1);
  block.Dwork(6).Data = rand(3,1);
  block.Dwork(7).Data = rand(3,1);
  block.Dwork(8).Data = rand(3,1);
  block.Dwork(9).Data = rand(3,1);
  block.Dwork(10).Data = rand(3,1);
 

  
  %block.DialogPrm(1).Data;
  
%endfunction

function Output(block)

  Kp = evalin('base', 'Kp');
  Ki = evalin('base', 'Ki');
  Kd = evalin('base', 'Kd');
  block.OutputPort(1).Data(1) = block.Dwork(10).Data(1) * Kp;
  block.OutputPort(1).Data(2) = block.Dwork(10).Data(2) * Ki;
  block.OutputPort(1).Data(3) = block.Dwork(10).Data(3) * Kd;
  
%endfunction

function Update(block)
  
  alpha = evalin('base', 'alpha');
  Kp = evalin('base', 'Kp');
  Ki = evalin('base', 'Ki');
  Kd = evalin('base', 'Kd');

  %% 前向传播
  block.Dwork(4).Data(1) =block.Dwork(1).Data' * block.InputPort(1).Data; % net(1) = W1 * [e(k),e(k-1),e(k-2),e(k-3)]
  block.Dwork(4).Data(2) =block.Dwork(2).Data' * block.InputPort(1).Data; % net(2) = W2 * [e(k),e(k-1),e(k-2),e(k-3)]
  block.Dwork(4).Data(3) =block.Dwork(3).Data' * block.InputPort(1).Data; % net(3) = W3 * [e(k),e(k-1),e(k-2),e(k-3)]
  
  %% sigmoid 计算
  % 使用循环遍历向量的每个元素
  for i = 1:3
     % 逐个计算 sigmoid 函数
     block.Dwork(5).Data(i) = 1 / (1 + exp(-block.Dwork(4).Data(i)));
  end
  %% 输出为sigmoid(net)

  block.Dwork(9).Data(1) = block.Dwork(6).Data' * block.Dwork(5).Data;
  block.Dwork(9).Data(2) = block.Dwork(7).Data' * block.Dwork(5).Data;
  block.Dwork(9).Data(3) = block.Dwork(8).Data' * block.Dwork(5).Data; % SUM = K * hidden_output
  
  
  %% sigmoid 计算
  % 使用循环遍历向量的每个元素
  for i = 1:3
     % 逐个计算 sigmoid 函数
     block.Dwork(10).Data(i) = 1 / (1 + exp(-block.Dwork(9).Data(i)));
  end
  %% 输出为sigmoid(SUM)


  %% 反向传播

  %% 比例
  dJ_dy = block.InputPort(1).Data(1); % e(k)
  dy_du = sign((block.InputPort(2).Data(1) - block.InputPort(2).Data(2))*(block.InputPort(3).Data(1) - block.InputPort(3).Data(2))); % sign[y(k)-y(k-1)/u(k)-u(k-1)]
  du_dP = Kp* (block.InputPort(1).Data(1) - block.InputPort(1).Data(2)); % Kp* e(k)-e(k-1)
  dP_dsum1 = block.Dwork(10).Data(1)*(1-block.Dwork(10).Data(1)); % P(1-P)
  dsum1_dk11 = block.Dwork(5).Data(1); % sigmoid(net1)，即hidden_output1
  dsum1_dk12 = block.Dwork(5).Data(2); % sigmoid(net2)，即hidden_output2
  dsum1_dk13 = block.Dwork(5).Data(3); % sigmoid(net2)，即hidden_output2
  
  dJ_dk11 = dJ_dy * dy_du * du_dP * dP_dsum1 *dsum1_dk11;
  dJ_dk12 = dJ_dy * dy_du * du_dP * dP_dsum1 *dsum1_dk12;
  dJ_dk13 = dJ_dy * dy_du * du_dP * dP_dsum1 *dsum1_dk13;

  dJ_dK1 = [dJ_dk11;dJ_dk12;dJ_dk13];
  
  %% 积分
  du_dI = Ki*block.InputPort(1).Data(1); % e(k)
  dI_dsum2 = block.Dwork(10).Data(2)*(1-block.Dwork(10).Data(2)); % I(1-I)
  dsum2_dk21 = block.Dwork(5).Data(1); % sigmoid(net1)，即hidden_output1
  dsum2_dk22 = block.Dwork(5).Data(2); % sigmoid(net2)，即hidden_output2
  dsum2_dk23 = block.Dwork(5).Data(3); % sigmoid(net2)，即hidden_output2

  dJ_dk21 = dJ_dy * dy_du * du_dI * dI_dsum2 *dsum2_dk21;
  dJ_dk22 = dJ_dy * dy_du * du_dI * dI_dsum2 *dsum2_dk22;
  dJ_dk23 = dJ_dy * dy_du * du_dI * dI_dsum2 *dsum2_dk23;

  dJ_dK2 = [dJ_dk21;dJ_dk22;dJ_dk23];


  %% 微分
  du_dD = Kd*block.InputPort(1).Data(1)+ block.InputPort(1).Data(3)-2*block.InputPort(1).Data(2); % e(k)+e(k-2)-2e(k-1)
  dD_dsum3 = block.Dwork(10).Data(3)*(1-block.Dwork(10).Data(3)); % D(1-D)
  dsum3_dk31 = block.Dwork(5).Data(1); % sigmoid(net1)，即hidden_output1
  dsum3_dk32 = block.Dwork(5).Data(2); % sigmoid(net2)，即hidden_output2
  dsum3_dk33 = block.Dwork(5).Data(3); % sigmoid(net2)，即hidden_output2

  dJ_dk31 = dJ_dy * dy_du * du_dD * dD_dsum3 *dsum3_dk31;
  dJ_dk32 = dJ_dy * dy_du * du_dD * dD_dsum3 *dsum3_dk32;
  dJ_dk33 = dJ_dy * dy_du * du_dD * dD_dsum3 *dsum3_dk33;

  dJ_dK3 = [dJ_dk31;dJ_dk32;dJ_dk33];

  %% 更新输出层
  block.Dwork(6).Data = block.Dwork(6).Data - alpha .* dJ_dK1;
  block.Dwork(7).Data = block.Dwork(7).Data - alpha .* dJ_dK2;
  block.Dwork(8).Data = block.Dwork(8).Data - alpha .* dJ_dK3;

  %% 隐藏层计算
  
  %% W1
  dsum1_dhidden_output1 = block.Dwork(6).Data(1);% k11
  dsum2_dhidden_output1 = block.Dwork(7).Data(1);% k21
  dsum3_dhidden_output1 = block.Dwork(8).Data(1);% k31
  dhidden_output1_dnet1 = block.Dwork(5).Data(1)*(1-block.Dwork(5).Data(1)); % hidden_output1 * (1-hidden_output1)
  dnet1_dw11 = block.InputPort(1).Data(1);% e(k)
  dnet1_dw12 = block.InputPort(1).Data(2);% e(k-1)
  dnet1_dw13 = block.InputPort(1).Data(3);% e(k-2)
  dnet1_dw14 = block.InputPort(1).Data(4);% e(k-3)

  dJ_dw11 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output1 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output1 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output1 )* dhidden_output1_dnet1 *dnet1_dw11;

  dJ_dw12 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output1 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output1 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output1 )* dhidden_output1_dnet1 *dnet1_dw12;

  dJ_dw13 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output1 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output1 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output1 )* dhidden_output1_dnet1 *dnet1_dw13;

  dJ_dw14 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output1 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output1 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output1 )* dhidden_output1_dnet1 *dnet1_dw14;

  %% W2
  dsum1_dhidden_output2 = block.Dwork(6).Data(2); % k12
  dsum2_dhidden_output2 = block.Dwork(7).Data(2); % k22
  dsum3_dhidden_output2 = block.Dwork(8).Data(2); % k32
  dhidden_output2_dnet2 = block.Dwork(5).Data(2)*(1-block.Dwork(5).Data(2)); % hidden_output2 * (1-hidden_output2)
  dnet2_dw21 = block.InputPort(1).Data(1);% e(k)
  dnet2_dw22 = block.InputPort(1).Data(2);% e(k-1)
  dnet2_dw23 = block.InputPort(1).Data(3);% e(k-2)
  dnet2_dw24 = block.InputPort(1).Data(4);% e(k-3)

  dJ_dw21 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output2 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output2 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output2 )* dhidden_output2_dnet2 *dnet2_dw21;

  dJ_dw22 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output2 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output2 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output2 )* dhidden_output2_dnet2 *dnet2_dw22;

  dJ_dw23 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output2 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output2 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output2 )* dhidden_output2_dnet2 *dnet2_dw23;

  dJ_dw24 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output2 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output2 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output2 )* dhidden_output2_dnet2 *dnet2_dw24;

  %% W3
  dsum1_dhidden_output3 = block.Dwork(6).Data(3); % k13
  dsum2_dhidden_output3 = block.Dwork(7).Data(3); % k23
  dsum3_dhidden_output3 = block.Dwork(8).Data(3); % k33
  dhidden_output3_dnet3 = block.Dwork(5).Data(3)*(1-block.Dwork(5).Data(3)); % hidden_output2 * (1-hidden_output2)
  dnet3_dw31 = block.InputPort(1).Data(1);% e(k)
  dnet3_dw32 = block.InputPort(1).Data(2);% e(k-1)
  dnet3_dw33 = block.InputPort(1).Data(3);% e(k-2)
  dnet3_dw34 = block.InputPort(1).Data(4);% e(k-3)

  dJ_dw31 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output3 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output3 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output3 )* dhidden_output3_dnet3 *dnet3_dw31;

  dJ_dw32 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output3 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output3 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output3 )* dhidden_output3_dnet3 *dnet3_dw32;

  dJ_dw33 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output3 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output3 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output3 )* dhidden_output3_dnet3 *dnet3_dw33;

  dJ_dw34 = dJ_dy * dy_du * (du_dP * dP_dsum1 * dsum1_dhidden_output3 ...
      + du_dI * dI_dsum2 * dsum2_dhidden_output3 + ...
      du_dD * dD_dsum3 * dsum3_dhidden_output3 )* dhidden_output3_dnet3 *dnet3_dw34;

  %% 更新
  dJ_dW1 = [dJ_dw11;dJ_dw12;dJ_dw13;dJ_dw14];
  dJ_dW2 = [dJ_dw21;dJ_dw22;dJ_dw23;dJ_dw24];
  dJ_dW3 = [dJ_dw31;dJ_dw32;dJ_dw33;dJ_dw34];
  
  block.Dwork(1).Data  = block.Dwork(1).Data - alpha * dJ_dW1;
  block.Dwork(2).Data  = block.Dwork(2).Data - alpha * dJ_dW2;
  block.Dwork(3).Data  = block.Dwork(3).Data - alpha * dJ_dW3;

%endfunction


