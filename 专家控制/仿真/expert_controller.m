function expert_controller(block)

  setup(block);
  
%end function

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
  block.Dwork(1).Data = 0;
  block.Dwork(2).Data = 0;
  block.Dwork(3).Data = 0;
  
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

  K_b = evalin('base', 'K_b');
  K_s = evalin('base', 'K_s');
  theta_1 = evalin('base', 'theta_1');
  theta_2 = evalin('base', 'theta_2');
  theta_m = evalin('base', 'theta_m');

  % 更新目标：F(k)(即block.Dwork(1).Data)，然后输出之
  % 更新步骤：
  % 首先，theta(k)输入(为InputPort(1).Data)，拿个变量记一下输入，简化书写
  theta_k = block.InputPort(1).Data;
  % theta(k)将其与theta(k-1)（Dwork变量）作差，得到Delta_theta(k)
  Delta_theta_k = theta_k -block.Dwork(2).Data;
  % 这样，Delta_Delta_theta也可以算了：
  Delta_Delta_theta = Delta_theta_k - block.Dwork(3).Data;
  % 至此，组成增量PID的三个关键，即theta,Delta_theta,Delta_Delta_theta都有了

  %% 砖家控制部分：
  % 若theta_k很大，则拉满输出力，不用管PID算法
  if abs(theta_k) >= theta_m
     block.Dwork(1).Data = sign(theta_k)* F_m;
  else
  % 若稍微比较大，则分两种情况讨论
     if ( abs(theta_k)>= theta_2 ) && ( abs(theta_k)< theta_m )
          
         % 1) 若乘积>0，则K=K_b再使用PID
         if theta_k * Delta_theta_k >0
              K = K_b;

         % 2) 若乘积<0，则再进一步分类讨论
         else
              %     i) 若两个Delta之乘积为正，则K=1
              if Delta_Delta_theta*Delta_theta_k > 0
                  K = 1;
              %     ii) 若两个Delta之乘积为负，则K=K_b
              else 
                  K = K_b;
              end
          end
      
    
      % 若较小，则分两种情况讨论
     elseif ( abs(theta_k)>= theta_1 ) && ( abs(theta_k)< theta_2 )
        
         % 1) 若乘积>0，则K=1再使用PID
         if theta_k * Delta_theta_k >0
              K = 1;
         % 2) 若乘积<0，则再进一步分类讨论
         else
            %     i) 若两个Delta之乘积为正，则K=K_s
            if Delta_Delta_theta*Delta_theta_k > 0
                K = K_s;
            %     ii) 若两个Delta之乘积为负，则K=1
            else 
                K = 1;
            end
         end

      % 若theta_k很小，则K=1再使用PID
     else 
         K = 1;
    
     end
      % 增量PID的F(k)更新公式：
      F_k = block.Dwork(1).Data + K*(K_p*Delta_theta_k + T/T_i*block.InputPort(1).Data + T_d/T *Delta_Delta_theta);
    
      % 更新Dwork参数们：
      % 最后，力的更新仍然要考虑上下限！
      if F_k> F_m
        block.Dwork(1).Data = F_m;
      elseif F_k< -F_m
        block.Dwork(1).Data = -F_m;
      else
        block.Dwork(1).Data = F_k;
      end
  end
  block.Dwork(2).Data = block.InputPort(1).Data; % theta_k参数就是输入本身
  block.Dwork(3).Data = Delta_theta_k; % Delta_theta 参数更新
  
%endfunction

