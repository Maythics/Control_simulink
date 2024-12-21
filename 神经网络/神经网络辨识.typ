#import "newtemplate.typ": *
#import "@preview/dashing-dept-news:0.1.0": newsletter, article
#import "@preview/cuti:0.2.1": show-cn-fakebold
#show: show-cn-fakebold

#show: apply-template.with(
  title:[*用神经网络进行系统辨识*],
  right_header:"ZJU CSE",
  left_header:"Control Science and Engineering",
  author: "章翰宇",
  ID:"3220104133",
  logo_path:"./images/CSE_logo.png",
  abstract: [如图所示二自由度机械臂模型（平面俯视图），$q_1$和$q_2$表示机械臂的两个关节角大小。请设计神经网络辨识方案，对该系统进行辨识（系统输入为$tau_1$，$tau_2$，输出为$q_1$，$q_2$）\
  + 利用已知系统得到辨识所需的输入输出数据\
  + 通过步骤1得到的数据来训练神经网络\
  + 对比原系统与神经网络辨识得到的系统是否一致
  ],
  keywords:[智能控制，神经网络，系统辨识],
  column: 1
  )

#set text(size: 12pt)

#figure(
  image("./images/fig_arm.png",width: 55%),
  caption: [物理动力系统示意图]
)

= 物理建模

#t 由于动力学方程为：

$ cases(m_(11) dot.double(q_1)+m_(12)dot.double(q_2)+c_(11) dot(q_1)+c_(12)dot(q_2) +g_1=tau_1\
m_(21) dot.double(q_1)+m_(22)dot.double(q_2)+c_(21) dot(q_1)+c_(22)dot(q_2) +g_2=tau_2  ) $

#t 有这些参数已知：
$ h_1=m_1 r_1^2 +m_2l_2^2+I_1\
h_2 =m_2 r_2^2 +I_2\
h_3 = m_2 l_1 r_2\
h_4 = m_1 r_1 + m_2 l_1\
h_5= m_2r_2 $

#t 为了便于在Simulink中仿真构造模型，干脆消去所有时变系数，整理成仅仅含有时不变系数$h_1 dots h_5$的如下两式

#set math.equation(block: true, numbering: "(1)")

- 描述$dot.double(q_1)$的：

$ (h_1 h_2 -h_3^2 cos^2 q_2)dot.double(q_1)=(2 h_2 h_3sin q_2 )dot(q_1)dot(q_2) +h_2h_3 sin q_2 dot(q_2)^2 +(h_3^2 cos q_2 sin q_2 +h_2 h_3 sin q_2)dot(q_1)^2\ -h_2 h_4 g cos q_1 + h_3 h_5 g cos q_2 cos(q_1+q_2) +h_2 tau_1 -(h_2+h_3 cos q_2)tau_2 $<eq1>

- 描述$dot.double(q_2)$的（上式基础上，由于Simulink连线可直接引$dot.double(q_1)$线，故保留$dot.double(q_1)$）：
$ h_2 dot.double(q_2) = tau_2 - h_5 g cos(q_1+q_2) -h_3 sin q_2 dot(q_1)^2- (h_2 + h_3cos q_2)dot.double(q_1) $<eq2>

#set math.equation(numbering: none)

#t 据此，建立如下的Simulink模型。解释一下该图：

#t 核心是中间偏右的几个积分块，代表$dot.double(q_1) -> dot(q_1)-> q_1$以及$dot.double(q_2) -> dot(q_2)-> q_2$两路（先不用看两个Guass白噪声，这是后续用的）。然后，$dot.double(q_1)$根据 @eq1 给出，把该式右边表达出然后除以系数$(h_1 h_2 -h_3^2 cos^2 q_2)$即得到$dot.double(q_1)$；$dot.double(q_2)$由 @eq2 得出，表达出右边再乘以$1\/h_2$的增益即可。

#t 左下角部分的两个白噪声那块部分，就代表$tau_1,tau_2$两个输入（当然也可以是别的形式，我这里为了训练，取白噪声叠加方波信号作为输入，取该输入的响应信号作为训练的数据集）

```matlab
h1 = 0.0308;h2 = 0.0106;
h3 = 0.0095;h4 = 0.2086;
h5 = 0.0631;g = 9.8;
```

#purple_theorem("Simulink")[#image("./images/simulink_model.png",width: 100%)]

#colbreak()

= 数学分析

== *思路概述*

#t 使用NARMA模型。分析该系统的阶数，由第一性原理，该模型的动力学都来自牛顿第二定律，本质上是二阶模型。而且，系统的演化（写成状态方程就能很明显看出）仅与输入量的本体（不依赖于输入的导数乃至高阶导数）以及输出量的本体和一阶导有关，故假设该系统为$Phi$，可以把该连续系统演化关系写为：
$ y(t+dd(t))=Phi_(dd(t))(y(t),dot(y)(t),u(t)) $

#t 其实写为状态方程应该用$u(t+dd(t))$，但是由于预测需要物理可实现，因此取$u(t)$

#t 将其离散化为：

$ y(k+1) = phi(y(k),y(k-1),u(k)) $

#t 题目中想用一个神经网络$f$来近似这个系统，可以写出这个虚拟的动力系统方程（采取串并联结构）：
$ hat(y)(k+1)= f(y(k),y(k-1),u(k)) $
#t 训练标准，采取均方误差最小。定义损失泛函$L$如下：
$ L(f):= sum_k e^2(k) =sum_k norm(hat(y)(k)-y(k))^2 $
#t 调参寻优即为泛函极值问题：求 $f^* = arg min L(f)$

== *具体表达*

#t （注：按照状态方程实际应该$y(k+1)$对应$u(k+1)$，但是对于一个系统预测其演化出于实际考量，可以写为我这里的样子，更加方便）

#t 将上一段的分析具体用本例的变量写出来，本系统的离散化为：

$ mat(q_1(k+1);q_2(k+1)) = phi(mat(q_1(k);q_2(k)),mat(q_1(k-1);q_2(k-1)),tau_1(k),tau_2(k)) $

#t 因此虚拟系统为：

$ mat(hat(q_1)(k+1);hat(q_2)(k+1))= f(mat(q_1(k);q_2(k)),mat(q_1(k-1);q_2(k-1)),tau_1(k),tau_2(k))  $

#t 变形整理成能在matlab里处理的形式，即：

#set math.equation(numbering: "(1)")

$ mat(hat(q_1)(k+1);hat(q_2)(k+1))= f(mat(q_1(k);q_2(k); q_1(k-1);q_2(k-1);tau_1(k);tau_2(k)))  $<eq3-NN>

#t 因此，看出待训练的神经网络的输入输出形状：输入为一个6维向量，输出为一个二维向量。

== *项目流程*

#t 今后只需要：

+ 在Simulink里把实际物理系统产生的序列数据$q_1(k),q_2(k),tau_1(k),tau_2(k)$导出来到工作空间
+ 将$q_1(k),q_2(k)$进行平移，得到$q_1(k-1),q_2(k-1),q_1(k+1),q_2(k+1)$这几个量，至此已经得到所有8个序列的数据
+ 根据 @eq3-NN 把这些数据拼成$6 times N$的输入向量input_data以及$2 times N$的ground truth标签向量label_data
+ 启动nftool应用，导入input_data以及label_data，根据该应用提示按按钮即可拟合
+ 将结果导出成Simulink模型，放回Simulink中测试，看看$tau_1,tau_2$变化后效果如何，若在各种输入下，神经网络输出和实际系统输出都很像，那么认为成功

#t 为了方便操作，编写一个data_preprocess.m脚本，自动完成上述的2,3两个步骤。

```matlab
%% data_preprocess.m
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
```

= 神经网络训练

== *离散化说明*

#t 首先，把连续的物理系统和离散的神经网络分离清楚。我规定Simulink中导出的To Workspace模块的采样时间为0.01秒，即，我的项目中，神经网络根据“此时的输出”、“0.01秒以前的输出”、“此时的输入”，来预测“0.01秒以后的输出”。

== *训练过程*

#t 导入数据入nftool如图：

#figure(
  image("./images/data_import.png",width: 55%),
  caption: [数据导入]
)

#t 然后，默认10层网络，默认70:15:15的数据划分。点击训练按钮（用莱文贝格-马夸特法），训练结束后有以下可视化结果：

#figure(image("./images/training.png",width: 55%))

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 2,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [#image("./images/train_plot.png",height: 25%)],[#image("./images/training_diagram.png",height: 25%)],
    [#image("./images/regression_plot.png",height: 25%)],
    [#image("./images/train_bar.png",height: 25%)],

    ),
    caption: [训练状态图]
)


= 辨识效果

== *串并联系统*

#t 如下图，上方为真实系统，下方安装神经网络虚拟系统，该神经网络（蓝色方块）接受一个六维输入$mat(q_1(k),q_2(k), q_1(k-1),q_2(k-1),tau_1(k),tau_2(k))^top$，输出一个预测的$mat(hat(q_1)(k+1),hat(q_2)(k+1))^top$
#purple_theorem("Simulink")[#image("./images/simulink_NN_model.png")]

#t 更改噪音的生成种子，并且修改输入形状如下（图中的1均指的是$q_1$，2均指的是$q_2$）：

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [输入],[输出],[放大图],
    [#image("./images/input1.png")],[#image("./images/test_result1.png")],[#image("./images/big1.png")],

  )
)

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [#image("./images/input2.png")],[#image("./images/test_result2.png")],[#image("./images/big2.png")],
    [#image("./images/input3.png")],[#image("./images/test_result3.png")],[#image("./images/big3.png")],
    [#image("./images/input4.png")],[#image("./images/test_result4.png")],[#image("./images/big4.png")],
    [#image("./images/input5.png")],[#image("./images/test_result5.png")],[#image("./images/big5.png")],
    [#image("./images/input6.png")],[#image("./images/test_result6.png")],[#image("./images/big6.png")],

    ),
    caption: [串并联结构网络拟合图]
)

#t 可见，在各种情况中，输出几乎都完全重合。说明神经网络的拟合效果还可以。但是，当本身的噪音选取过大或者恰好输入导致结果发散的话，那么预测误差也会发散。

== *并联模型*

#t 我尝试用一下并联模型，搭建模型如下：唯一的修改就是右下角部分，$q_1,q_2$这两路的来源，现在改为前5秒来自真实系统，后5秒来自神经网络自己的输出。这是模拟一开始学习，后来神经网络与原系统脱离的情景。（前5秒必须来自原始系统的原因是神经网络初始的输出是很奇怪的，修改了$q_1(0)$并不能让神经网络意识到这一点，所以先得跟踪一会）

#purple_theorem("Simulink")[#align(center)[#image("./images/par_simulink_model.png",width: 80%)]]


#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [输入],[输出],[放大图],
    [#image("./images/par_input1.png")],[#image("./images/par_result1.png")],[#image("./images/par_big1.png")],
    [#image("./images/par_input2.png")],[#image("./images/par_result2.png")],[#image("./images/par_big2.png")],


    ),
    caption: [并联结构网络拟合图]
)

#t 可见，这个系统的非线性性非常强，随着几轮迭代，输出越差越大。这类似于蝴蝶效应。

= 串并联网络调试分析

#t 以下，企图分析网络结构以及训练数据对结果的影响，因此做了这些测试，并且打印出误差大小供辅助判断。在测试时，打开了在$dot.double(q_1),dot.double(q_2)$处的高频白噪音，这样的话，能从误差的频率看出，误差究竟来自于哪里：如果误差频率很高，那说明网络基本预测准了，只是收到这些未知噪音的扰动；如果误差很大而且频率慢，则说明网络预测的还不够准，偏差来源于拟合不佳。

=== 原网络预测阶跃/三角输入响应的误差

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [输入],[响应],[误差],
    [#image("./images/step_input.png")],[#image("./images/step_result.png")],[#image("./images/step_err.png")],
    [#image("./images/step_input2.png")],[#image("./images/step_result2.png")],[#image("./images/step_err2.png")],
    [#image("./images/sin_input.png")],[#image("./images/sin_result.png")],[#image("./images/sin_err.png")],
    [#image("./images/sin_input2.png")],[#image("./images/sin_result2.png")],[#image("./images/sin_err2.png")],

    ),
    caption: [串并联结构网络拟合效果]
)<table-predict>

#t 可见，实际上误差的量级在$10^(-4)$，即，相对误差在0.1%内，说明刚才训练出来的网络的确有预测效果。此外，从上方的误差err图发现，误差的频率远远高于两个广义坐标的变化频率，这可以说明这个误差类似于随机误差而非系统性的误差。分析上图知，在初始时误差很大，是因为神经网络不清楚初始情况，而在阶跃响应的第二秒有较大的误差，正是因为阶跃的突变造成瞬间没跟上导致的预测失败。

== *减少训练数据的个数重新测试*

#t 减少到使用白噪声输入产生的1000以及100个数据，重新训练两个新网络，重新用阶跃输入测试（见 @table-predict 表格的第一行的那个阶跃输入），看看能否跟踪。

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [样本数],[响应],[误差],
    [10000],[#image("./images/step_result.png")],[#image("./images/step_err.png")],
    [1000],[#image("./images/step_less_result.png")],[#image("./images/step_less_err.png")],
    [100],[#image("./images/step_small_result.png")],[#image("./images/step_small_err.png")],

    ),
    caption: [串并联结构网络（不同训练样本数）拟合效果]
)
#t 发现1000个数据训练的挺不错，和10000个数据训练的在一个数量级，误差稍稍大一点点。但是100就很不理想，明显是系统误差（即神经网络预测的函数不对），因为如果噪音来自扰动，应该频率很高（如前面所述）。此外，这里的误差量级很大，也足以说明问题。


== *更改网络结构的训练*

#t 同理，都使用1000个数据点（产生自白噪音输入），但是一个是用10层，一个是用6层，一个是用2层，看看效果如何。测试效果时，仍用阶跃输入（见 @table-predict 表格的第一行的那个阶跃输入），看看能否跟踪。


#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [网络 \ 层数],[响应],[误差],
    [10],[#image("./images/step_less_result.png")],[#image("./images/step_less_err.png")],
    [6],[#image("./images/step_6_result.png")],[#image("./images/step_6_err.png")],
    [2],[#image("./images/step_2_result.png")],[#image("./images/step_2_err.png")],

    ),
    caption: [串并联结构网络（不同层数）拟合效果]
)

#t 发现10层数据训练的很不错但6层误差稍大一些，但是仍然在$10^(-4)$量级，基本可以接受。但是当网络仅仅有2层时，不是很理想，从误差的变化频率也可以看出来（和层数多的比变慢了许多），证明误差主要来源不是随机的噪音，而是来源于系统性的（即神经网络预测的函数明显不对），误差量级很大，有$10^(-2)$。


= 加入噪声的数据训练出的网络

#t 假设考虑到实际情形，从机械臂的传感器得来的角度是有噪音的（体现在两个角加速度那一项受到额外的扰动），具体表现就是打开之前流程图里$dot.double(q_1)$和$dot.double(q_2)$处的两个白噪声，这样得到的数据再进行训练。之所以情况不同，就是原本神经网络学的是一个力学函数关系，但现在不是一个完美的函数关系，会有一些噪点，网络可能学坏。


#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [输入],[输出],[放大图],
    [#image("./images/noise_input1.png")],[#image("./images/noise_result1.png")],[#image("./images/noise_big1.png")],
    [#image("./images/noise_input2.png")],[#image("./images/noise_result2.png")],[#image("./images/noise_big2.png")],
    ),
    caption: [串并联结构，用带噪音的数据训练的网络拟合图]
)

#t 可以看到，效果稍微差一些，仔细看放大图能发现明显的偏离。在上表第二种输入下，对比旧网络与现在这个网络拟合表现如下。用含噪音的数据训练出来效果略差一些（也可能是运气问题）。

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 2,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [之前的网络],[含噪音数据训练的网络],
    [#image("./images/compare.png",height:20%)],[#image("./images/noise_big2.png",height:20%)],
    ),
    caption: [用带噪音与不带噪音的数据训练的两个网络对比]
)

= 代码附件与运行说明

> 使用的Matlab版本为2023b

附件含有：\
> arm_model.slx 仿真模型 \
> robotarm_param.m 含有参数\
> data_preprocess.m 数据处理助手\

#t 测试时，请先运行robotarm_param.m文件（使得workspace中有参数的数据）然后即可运行slx文件。运行完成后，运行data_preprocess.m文件即可获得数据集，然后打开nftool导入input_data和label_data两个量即可训练。

#t slx文件中含有两组，一组是串并联，一组是并联（注释），一共提供了6个神经网络（分别是不同的训练样本个数训练出来的10层网络，以及用同样的1000个数据训练但层数不同的网络），测试时可以替换模块进去。

#text(red.darken(10%))[#t 如有疑问，请访问 https://github.com/Maythics/Control_simulink.git]

