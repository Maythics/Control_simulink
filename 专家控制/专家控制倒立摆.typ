#import "newtemplate.typ": *
#import "@preview/dashing-dept-news:0.1.0": newsletter, article
#import "@preview/cuti:0.2.1": show-cn-fakebold
#show: show-cn-fakebold

#show: apply-template.with(
  title:[*专家控制*],
  right_header:"ZJU CSE",
  left_header:"Control Science and Engineering",
  author: "章翰宇",
  ID:"3220104133",
  logo_path:"./images/CSE_logo.png",
  abstract: [车载倒立摆系统，一辆小车在水平轨道上移动，小车上有一个可绕固定点转动的倒立摆。控制小车在水平方向的移动可使摆杆维持直立不倒。比较离散PID控制器以及专家控制方法，针对不同的初始夹角，给出专家PID控制的结果。],
  keywords:[智能控制，专家控制],
  column: 1
  )

#set text(size: 12pt)

#text(red.darken(10%))[如有疑问，请访问 https://github.com/Maythics/Control_simulink.git]

= 模型建立

#t 如下图，忽略车轮与地面的摩擦力等阻力，可推导出车载倒立摆的动力学方程（分别为大车水平分量，小球水平和竖直分量）：

$ cases(F- N cos(theta) = M dot.double(x) \
N sin theta = m(dot.double(x) +l(cos theta dot.double(theta)-sin theta dot(theta)^2))\
N cos theta -m g = m l (-cos theta dot(theta)^2 - sin theta dot.double(theta))
) $

#figure(
  image("./images/cart_fig.png", width: 50%),
  caption:[倒立摆模型]
)

#t 消去变量$N$，得到：

$
  cases((M+m)dot.double(x) + m l cos theta dot.double(theta)-m l sin theta dot(theta)^2 =F\
 m l^2 dot.double(theta) + m l dot.double(x) cos theta - m g l sin theta = 0
  )
$

#t 为了便于画方块图，将$x$的二阶导数移到左边：

$ cases(dot.double(x) = ((M+m)/(m l cos theta)-1/l cos theta)^(-1)(tan theta dot(theta)^2+F/(m l cos theta) - g/l sin theta)\
dot.double(theta) = g/l sin theta - 1/l cos theta dot.double(x)
) $

#t 下面是方块图：

#purple_theorem("Simulink")[
#figure(
  image("./images/model_PID.png", width: 100%), caption: [Simulink模型图（离散PID版本）]
)]

#t 框图解释：核心是中间两路$dot.double(theta) arrow dot(theta) arrow theta$以及$dot.double(x) arrow dot(x) arrow x$。

#t 其中，$dot.double(theta)$的值根据$g/l sin theta - 1/l cos theta dot.double(x)$给出，因此是两部分的相减，下面一部分，来自$theta$，作用$sin$后再乘以Gain得到；上面一部分来自$cos$作用后的$theta$与$dot.double(x)$的乘积。

#t $dot.double(x)$比较复杂，表达式为$((M+m)/(m l cos theta)-1/l cos theta)^(-1)(tan theta dot(theta)^2+F/(m l cos theta) - g/l sin theta)$，因此，框图最上方的coef块即计算了前面这个很大的系数，而框图左侧的三个部分相加减，即$tan theta dot(theta)^2,F/(m l cos theta), g/l sin theta$这三部分。

#t disc_PID_controller部分使用S-Function实现。

= 控制器设计

#red_theorem("Warning")[我使用的均是Level 2的S-Function]

#t 以下为离散PID的实现代码主要部分，设置了三个内置变量：

#block(
  fill: luma(245),
  inset: 5pt,
  radius: 4pt,
  [
```matlab
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

  %block.DialogPrm(1).Data;

function Output(block)

  block.OutputPort(1).Data = block.Dwork(1).Data; % 输出即为F(k)

function Update(block)

  % 更新目标：F(k)(即block.Dwork(1).Data)，然后输出之
  % 更新步骤：
  % 首先，theta(k)输入(为InputPort(1).Data)，将其与theta(k-1)（Dwork变量）作差，得到Delta_theta(k)
  Delta_theta_k = block.InputPort(1).Data-block.Dwork(2).Data;
  % 这样，Delta_Delta_theta也可以算了：
  Delta_Delta_theta = Delta_theta_k - block.Dwork(3).Data;
  % 至此，组成增量PID的三个关键，即theta,Delta_theta,Delta_Delta_theta都有了

  % 增量PID的F(k)更新公式：
  F_k = block.Dwork(1).Data + K*(K_p*Delta_theta_k + T/T_i*block.InputPort(1).Data + T_d/T *Delta_Delta_theta);

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
```
]
)

#t 也可以使用专家控制方法，和离散PID唯一的区别是，增加了一些判断语句，分类讨论$K$的取值，但是上面的PID则是固定的$K$取值（我默认设置为1）

#block(
  fill: luma(245),
  inset: 5pt,
  radius: 4pt,
  [
```matlab
%% 专家控制额外部分：
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
```
])

= 效果对比分析

#t 准备测试，使用题中建议的参数列表：

#block(
  fill: luma(245),
  inset: 5pt,
  radius: 4pt,
  [```matlab
%% 离散PID参数
m = 0.5; M = 1; l = 0.5; g = 9.8;
T = 0.0001; % sample period
K_p = 200; T_i = 0.001; K = 1;
T_d = 10; F_m =25; % max input
%% 专家控制部分参数
theta_1 = 0.1; theta_2 = 0.3;
theta_m = 0.5; K_s = 1; K_b = 1.3;
```]
)
#t 框图如下图 @fig-compare，把离散PID系统控制的一个系统与专家控制改进的系统，其输出值汇总到一个Scope中进行对比，分析区别：


#figure(
  table(
    stroke: luma(220)+0.1em,
    columns: 3,
    gutter: 0.2em,
    inset: 0pt,
    [#image("./images/input.png")],[#image("./images/x_change.png")],
    [#image("./images/theta_change.png")]
  ),caption: [1. 输入 2. 小车位移 3. 倒立摆角度]
)

#t 可见，在该种参数下，两方法都可以使得倒立摆的摆角稳定在0度左右，并且调节时间都很快（1秒以内）。但是，专家控制的倒立摆角度无超调，调节时间更短，而且使用的能耗更小（参见输入曲线），最后由于调节产生的小车速度也更小。因此，从快速性和稳定性来分析，专家控制更胜一筹。

#purple_theorem("Simulink")[
#figure(image("./images/compare.png", width: 100%), caption: [对比框图])<fig-compare>
]

= 不同初始条件的专家控制

#t 下面，探索不同初值$theta(0)$下，专家控制效果是否仍然较好。首先证明如下断言：

#blue_theorem([*Claim*], [_*Proof*_ #t 以车为参考系，分析小球受力，由于非惯性系中惯性力水平方向向左，且最大为$(m F_m)\/M$，又受到竖直向下的重力$m g$，小球所受杆子的支撑力沿杆径向，因此，如果重力与惯性力合力，与竖直方向的夹角大于杆子与竖直方向的夹角，则所受合力不可能使得小球回归平衡，由几何知识得，此临界角$theta_0^ast = arctan(F_m/(M g))$])[$exists  theta_0^ast in [0,pi/2]$，使得，若初始条件满足 $theta(0)=theta_0$，其中$forall theta_0> theta_0^ast$，则，不存在一组合适的参数，使得$theta(infinity) = 0$]

#t 因此，无需测试初始角度过大的倒立摆系统，因为在此题框架下，存在一些过大初值，不能控制成功的情况。

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 0em,
    align: center+horizon,
    [$ theta(0) $],[$ (theta_1;theta_2;theta_m;K_s;K_b) =(0.1,0.3,0.5,1,1.3) $],[$ (theta_1;theta_2;theta_m;K_s;K_b) =(0.1,0.2,0.3,2,3) $],[$  (3pi)/10 $],
    [#image("./images/0.3pi_old.png")],[#image("./images/0.3pi_new.png")],
    [$ (pi)/4 $],
    [#image("./images/45degree_old.png")],[#image("./images/45degree_new.png")],
    [$ (pi)/5 $],[#image("./images/0.2pi_old.png")],[#image("./images/0.2pi_new.png")],
    [$ (pi)/10 $],[#image("./images/0.1pi_old.png")],[#image("./images/0.1pi_new.png")]
  )
)


#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 0em,
    align: center+horizon,
    [$ theta(0) $],[$ (theta_1;theta_2;theta_m;K_s;K_b) =(0.2,0.4,0.6,1,1.2) $],[$ (theta_1;theta_2;theta_m;K_s;K_b) =(0.2,0.5,0.8,1.8,2) $],[$  (3pi)/10 $],
    [#image("./images/0.3pi_new2.png")],[#image("./images/0.3pi_new3.png")],
    [$ (pi)/4 $],
    [#image("./images/45degree_new2.png")],[#image("./images/45degree_new3.png")],
    [$ (pi)/5 $],[#image("./images/0.2pi_new2.png")],[#image("./images/0.2pi_new3.png")],
    [$ (pi)/10 $],[#image("./images/0.1pi_new2.png")],[#image("./images/0.1pi_new3.png")]
  ),caption: [不同初始值下不同参数对应控制效果]
)

= 抗扰动测试

== *白噪声*

#t 模拟当小车在运动时，$dot.double(theta)$收到高斯白噪声的影响后，是否还能够较好维持在$theta = 0$附近

#purple_theorem("Simulink")[#image("./images/noise_model.png",width: 100%)]

#t 此处，取的专家控制参数为：$ (theta_1;theta_2;theta_m;K_s;K_b) =(0.1,0.3,0.5,1,1.3) $
#t 取的初始角为$ theta(0) = pi/4 $
#t 模型如上图，$dot.double(theta)$处加入了噪声，下表为不同噪声功率时，两种控制方法的效果对比。


#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 4,
    gutter: 0em,
    inset: 1pt,
    align: center+horizon,
    [功率],[输入],[位移],[角度],
    [0.5],[#image("./images/noise_input.png")],[#image("./images/noise_x.png")],[#image("./images/noise_theta.png")],

    [1],[#image("./images/bignoise_input.png")],[#image("./images/bignoise_x.png")],[#image("./images/bignoise_theta.png")],

    [5],[#image("./images/biggernoise_input.png")],[#image("./images/biggernoise_x.png")],[#image("./images/biggernoise_theta.png")],

  ),caption: [白噪音下的控制效果对比]
)

#t 可见，专家控制在有噪音的情况下，控效果也比离散PID要更好一些

== *阶跃扰动*

#t 模拟当小车在运动时，球体收到榔头的冲激导致速度$dot(theta)$突变的情况，此种扰动$dot.double(theta)$可视为无穷，比高斯白噪声影响更大。

#t 此处，取的专家控制参数为：$ (theta_1;theta_2;theta_m;K_s;K_b) =(0.1,0.3,0.5,1,1.3) $
#t 取的初始角为$ theta(0) = pi/4 $
#t 模型如下图，保留了$dot.double(theta)$处的功率为1的高斯白噪声，并新增加$dot(theta)$处，0.8秒时加入了大小为$pi$的角速度阶跃。

#purple_theorem("Simulink")[#image("./images/stepnoise_model.png",width: 100%)]

#figure(
  table(
    stroke: luma(220)+0.1em,
    columns: 3,
    gutter: 0.2em,
    inset: 0pt,
    [#image("./images/stepnoise_input.png")],[#image("./images/stepnoise_x.png")],
    [#image("./images/stepnoise_theta.png")]
  ),caption: [榔头敲击下，1. 输入 2. 小车位移 3. 倒立摆角度]
)

#t 可见，抗冲激效果还可以。

= 不同平衡位置测试

#t 之前已经测试了平衡点为$theta = 0$的情况，下面测试平衡点为$theta = 0.1 pi$与$theta = -0.1 pi$的情形。

#t 取的专家控制参数为：$(theta_1;theta_2;theta_m;K_s;K_b) =(0.1,0.3,0.5,1,1.3)$

#t 初始角度为 $theta(0)=pi/4$

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 4,
    gutter: 0em,
    inset: 1pt,
    align: center+horizon,
    [平衡点],[输入],[位移],[角度],
    [$ 0.1 \ pi $],[#image("./images/0.1_input.png")],[#image("./images/0.1_x.png")],[#image("./images/0.1_theta.png")],

    [$ -0.1 \ pi $],[#image("./images/-0.1_input.png")],[#image("./images/-0.1_x.png")],[#image("./images/-0.1_theta.png")],

  ),caption: [不同平衡点]
)<fig-0.1-equilibrium>

#t 可见，该专家控制规则也有缺点，在平衡角度为正时没问题，但平衡角度为负时，由于初始角度为正，因此一开始差距很大，导致输入过大，会出现极大的超调，后来刹车已经来不及了，因此稳定性低于PID，事实上，当平衡角度过负时（比如$-0.2pi$），专家控制就会失效，但是PID仍然能够成功（不考虑任何噪声）

= 新专家控制

== *动机分析*

#t 目前的专家控制和离散PID控制均对于较大的平衡点做的不好，尤其是专家控制。虽然目前的专家控制在最终平衡点为0或附近时，能有非常好的控制效果（无论是速度方面还是能耗方面），但是对于偏离0较大的负角度，就无法平衡了，出现很大超调（见 @fig-0.1-equilibrium）即不稳定的预兆，事实上，当设置$theta(0) =pi/4$但是最终平衡位置为$-0.18 pi$时，则专家控制失效，离散PID仍能成功；当最终平衡位置为$0.18 pi$时，两者均失败：（注：此处均加入了白噪音和阶跃冲激，没噪音时这个角度是能成功的）

#figure(
   table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 2,
    gutter: 0em,
    inset: 1pt,
    align: center+horizon,
    [#image("./images/expert_disaster.png", width: 100%)],[#image("./images/disaster.png", width: 100%)]

  ),caption:"当前两种方案控制的失败案例"
)

== *新专家控制思路*

#t 之前的专家控制之所以失效，是因为在差的很远时调大了$K$值，这会导致输入过头了无法刹车。因此，为保险起见，使用普通PID较好，但是普通PID的问题在于，误差太大时，输入还不够大，这又招致调节能力不足。\
#t 因此，新的方案是，在误差较大时，仅仅使用比例控制而禁用积分微分，但是调大比例系数，这一方面能避免旧专家控制的输入过头问题，又缓解了普通PID控制的输入不足问题。控制器设计如下，仅仅在离散PID的基础上做如下修改：

#block(
  fill: luma(245),
  inset: 5pt,
  radius: 4pt,
  [```matlab
 % 增量PID的F(k)更新公式：
  if Delta_Delta_theta+Delta_theta_k >= 0.0001
      F_k = block.Dwork(1).Data + 100*K*(K_p*Delta_theta_k);
  else
      F_k = block.Dwork(1).Data + K*(K_p*Delta_theta_k + T/T_i*block.InputPort(1).Data + T_d/T *Delta_Delta_theta);
  end
```]
)

== *新专家控制方法效果*

#t 测试时，提供三种方法（离散PID、之前的专家控制、新专家控制）对比，发现新的专家控制在另外两种方法都发散的情形下，能够镇定系统。\
#t 注意，测试时加入了功率为0.1的白噪音和在0.8秒时施加的大小为$pi$的角速度阶跃冲激，见下图在0.8秒处遭受较大的冲激后仍能恢复，证明新专家控制方案可行。下图中，由上到下的三个系统分别为旧专家、PID、新专家，共用一个scope查看信号。

#purple_theorem("Simulink")[#image("./images/3methods.png", width: 60%)]

#figure(
   table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 2,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [平衡点$-0.18 pi$],[平衡点$ 0.18 pi$],
    [#image("./images/newexpert_input2.png", width: 100%)],[#image("./images/newexpert_input1.png", width: 100%)],
    [#image("./images/newexpert_x2.png", width: 100%)],[#image("./images/newexpert_x1.png", width: 100%)],
    [#image("./images/newexpert_theta2.png", width: 100%)],[#image("./images/newexpert_theta1.png", width: 100%)],


  ),caption:"前两种方案控制的失败时，新专家控制可行"
)

= 代码附件与运行说明

> 使用的Matlab版本为2023b

附件含有：\
> cart_pendulum.slx 仿真模型 \
> param_for_cart.m 含有参数\
> expert_controller.m、new_expert_controller.m、disc_PID_controller.m 三个不同的控制器S-function文件

测试时，请先运行param_for_cart.m文件（使得workspace中有参数的数据，这样Simulink以及S-function文件才能读到参数数值）然后即可运行slx文件。
