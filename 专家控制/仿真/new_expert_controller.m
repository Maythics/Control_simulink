function new_expert_controller(block)

  setup(block);
  
%endfunction

function setup(block)
  
  block.NumDialogPrms  = 3;
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 1;
  
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
  block.NumDworks = 3;
  
  % 一个内置更新量为F(k)
  block.Dwork(1).Name = 'F'; 
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

  % 一个内置更新量为theta(k)
  block.Dwork(2).Name = 'theta'; 
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;

  % 一个内置更新量为Delta_heta(k)
  block.Dwork(3).Name = 'Delta_theta'; 
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;

%endfunction

function InitConditions(block)

  %% Initialize Dwork
  block.Dwork(1).Data = block.DialogPrm(1).Data;
  block.Dwork(2).Data = block.DialogPrm(2).Data;
  block.Dwork(3).Data = block.DialogPrm(3).Data;
  
  %block.DialogPrm(1).Data;
  
%endfunction

function Output(block)

  block.OutputPort(1).Data = block.Dwork(1).Data; % 输出即为F(k)
  
%endfunction

function Update(block)
  
  T = evalin('base', 'T');
  K_p = evalin('base', 'K_p');
  T_d = evalin('base', 'T_d');
  T_i = evalin('base', 'T_i');
  F_m = evalin('base', 'F_m');
  K = evalin('base', 'K');
  % 更新目标：F(k)(即block.Dwork(1).Data)，然后输出之
  % 更新步骤：
  % 首先，theta(k)输入(为InputPort(1).Data)，将其与theta(k-1)（Dwork变量）作差，得到Delta_theta(k)
  Delta_theta_k = block.InputPort(1).Data-block.Dwork(2).Data;
  % 这样，Delta_Delta_theta也可以算了：
  Delta_Delta_theta = Delta_theta_k - block.Dwork(3).Data;
  % 至此，组成增量PID的三个关键，即theta,Delta_theta,Delta_Delta_theta都有了

  % 增量PID的F(k)更新公式：
  if Delta_Delta_theta+Delta_theta_k >= 0.0001
      F_k = block.Dwork(1).Data + 100*K*(K_p*Delta_theta_k);
  else
      F_k = block.Dwork(1).Data + K*(K_p*Delta_theta_k + T/T_i*block.InputPort(1).Data + T_d/T *Delta_Delta_theta);
  end
  % 更新Dwork参数们：
  % 力的更新要考虑上下限：
  if F_k> F_m
    block.Dwork(1).Data = F_m;
  elseif F_k< -F_m
    block.Dwork(1).Data = -F_m;
  else
    block.Dwork(1).Data = F_k;
  end
  block.Dwork(2).Data = block.InputPort(1).Data; % theta_k参数就是输入本身
  block.Dwork(3).Data = Delta_theta_k; % Delta_theta 参数更新
  
%endfunction

