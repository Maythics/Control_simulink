#import "newtemplate.typ": *
#import "@preview/dashing-dept-news:0.1.0": newsletter, article
#import "@preview/cuti:0.2.1": show-cn-fakebold
#show: show-cn-fakebold

#show: apply-template.with(
  title:[*项目作业 平衡车控制*],
  right_header:"ZJU CSE",
  left_header:"Control Science and Engineering",
  author: "章翰宇",
  ID:"3220104133",
  logo_path:"./images/CSE_logo.png",
  abstract: [平衡车在日常生活中很常见，它脱胎于倒立摆的控制问题。本项目以平衡车为对象，将倒立摆问题深化，从一维导轨拓展到二维平面上的运动，从单一变量摆角的控制，拓展到三个变量的控制。项目中，使用模糊控制以及神经网络PID控制方法，设计出一种拥有比普通PID更平滑控制效果，且能在模型参数大幅度变化后仍然成功控制的方案。最终方案为：用两个模糊控制器与一个神经网络PID控制器，取代所有的普通PID控制器，达到更优的效果。],
  keywords:[智能控制，平衡车，神经网络控制，模糊控制],
  column: 1
  )

#set text(size: 12pt)

#figure(
  table(
    stroke: oklab(98.02%, 0.001, -0.001, 0%)+0.1em,
    columns: 2,
    gutter: 5em,
    inset: 0em,
    align: center+horizon,
    [#image("./images/balance_car.png",height: 20%)],[#image("./images/wheeltec.png",height: 15%)],
  ),
  caption: [现实中的平衡小车，往往采用普通的PID控制策略]
)


= 与“车载倒立摆”的区别

#t 之前的车载倒立摆系统，考虑的固定在导轨上的小车（SISO），在本例中，没有导轨，有两个轮子（甚至可以转弯），两个轮子的转速不同将造成偏航角的变化，偏航角也将纳入被控变量考虑。之前，输入系统的控制信号是水平力$F$，此处，输入信号是两个电各自的加速度，和实际开发平衡车产品中的控制问题条件一致。

#t 之前倒立摆作业没有控制住位移，仅仅控制了摆角，此处我们的目标是控制在二维平面上运动并且稳定下来，因此本质是$x$的跟踪问题，但是依赖于摆角的变化，需要协调两者之间的关系。

#t 因此，此处的力学分析要更加复杂全面一些，由于是二维运动，是MIMO问题，操控变量有两个，被控变量有3个（位移、倾角、航向）。


= 模型建立

== *符号说明*

#align(center)[
  #tablex(
  fill: (x, y) => if y==0 {blue.lighten(80%)} else{white},
  stroke:1.5pt + oklab(61.76%, -0.123, 0.032, 48.6%), //+luma(180),
  columns: 2,
  align: left + horizon,
  auto-vlines: true,
  auto-hlines: true,
  // repeat-header: true,
  inset: 6pt,
  /* --- header --- */
  [*Symbol*],[*Explanation*],
  [$m$],[单个车轮的质量],
  [$M$],[车体的质量],
  [$x$],[两个轮子中心点的坐标],
  [$d$],[两个轮子之间的距离],
  [$J_p$],[车体绕质心俯仰转动的转动惯量],
  [$J_delta$],[车体绕质心偏航转动的转动惯量],
  [$I$],[一个轮子绕其轴转动的转动惯量],
  [$delta$],[整体的偏航角],
  [$theta_p$],[车体的俯仰角（前倾程度）],
  [$l$],[车体的质心到轮轴的距离],
  [$x_L,x_R$],[左，右轮子的水平方向位移],
  [$H_L,H_R$],[车体与左，右轮子之间的水平作用力大小],
  [$V_L,V_R$],[车体与左，右轮子之间的竖直作用力大小],
  [$H_(f L),H_(f R)$],[左，右车轮与地面之间的摩擦力大小],
  [$T_L,T_R$],[左，右轮子处电机施加的矩大小],

)
]
== *轮子受力分析*

#figure(image("./images/wheel.png",width: 30%))

分析两个轮子的受力如上图，分别有合力公式与合力矩公式：
$
cases(dot.double(x)_R m = H_(f R)-H_R \
dot.double(theta) I = T_R - H_(f R) r  )
$
由于$H_(f R)$是地面给的摩擦力，是隐变量的地位，因此消去之：
$ dot.double(x)_R m = (T_R -dot.double(theta) I )/r - H_R $
根据无滑滚动约束条件，$dot(x_R) = r dot(theta)$，因此将$theta$消去，上式成为：
$ dot.double(x)_R (m+I/r^2) = (T_R  )/r - H_R $
同理：
$ dot.double(x)_L (m+I/r^2) = (T_L  )/r - H_L $
由于$x = (x_R+x_L)/(2)$，因此：
#set math.equation(block: true, numbering: "(1)")
$ dot.double(x)(m + I/r^2) = (T_R+T_L)/(2r) - (H_L+H_R)/(2)  $
$
(dot.double(x_L)-dot.double(x_R))(m+I/r^2) = (T_L - T_R)/r - (H_L - H_R)
$<eq-diff>
#set math.equation(numbering: none)


== *车体受力分析*

#figure(image("./images/body.png",width: 35%))
$
cases(M dv((x+l sin theta_p),t,2) = H_L +H_R \

M dv((l cos theta_p),t,2) = V_L +V_R -M g
\

J_p dot.double(theta)_p = (V_L+V_R) l sin theta_p - (T_R +T_L) - (H_R +H_L) l cos theta_p
)
$
#set math.equation(block: true, numbering: "(1)")
因此，结合轮子的受力分析消除内力$(H_L+H_R)$以及$(V_L +V_R)$部分，就能得到式子：
$
  cases((M+2m +(2I)/r^2)dot.double(x) - (T_R+T_L)/r + M l dot.double(theta)_p cos theta_p-M l dot(theta)_p^2 sin theta_p=0 \
(J_p/(M l)+l)dot.double(theta)_p + dot.double(x) cos theta_p -g sin theta_p + (T_L+T_R)/(M l) = 0

  )
$<eq-main>
#set math.equation(block: true, numbering: none)
== *偏航转向分析*

回顾$d$代表两个轮子之间的距离，因此：
$
  J_delta dot.double(delta) = d/2 (H_L-H_R)
$
#figure(image("./images/turning.png",width: 25%))
由示意图的几何关系可得：
$
  dot(delta) = (dot(x_L)-dot(x_R))/d
$
把这个代入上式，并且结合 @eq-diff，得到$delta$演化的动力学方程：
#set math.equation(block: true, numbering: "(1)")
$
  dot.double(delta) = 1/(r(m d+(I d)/r^2 + (2J_delta)/d )) (T_L- T_R)
$<eq-turning>

== *Simulink建模*

#t 将@eq-main 整理成为容易建模的形式如下（@eq-turning 就不需要变了）：
$
[M l cos^2 theta_p-(M+2m +(2I)/r^2)(J_p/(M l)+l)]dot.double(theta)_p = M l sin theta_p cos theta_p dot(theta)_p^2 - g sin theta_p (M+ 2m +(2I)/r^2) + \
[(cos theta_p)/r + ((M + 2m +(2I)/r^2))/(M l)](T_R+T_L)
$
$
[(J_p/(M l)+l)(M+2m +(2I)/r^2)-M l cos^2 theta_p]dot.double(x) =  M l sin theta_p (J_p/(M l )+l) dot(theta)_p^2 -M g l cos theta_p sin theta_p + \ [cos theta_p + ((J_p/(M l )+l))/r](T_R+ T_L)
$
#set math.equation(block: true, numbering: none)

#t 在绘制Simulink框图时，最后$T_R$与$T_L$这两者由于代表电机输出转矩的大小，不好直接控制，因此转化为两个车轮在无摩擦情况下的加速度:$ a_L =r/I T_L ,a_R =r/I T_R$。

#purple_theorem([*Simulink*])[#image("./images/dynamic.png")]

#t 上图为建立的动力学系统框图，体现了整个模型的结构，整个开环对象，可以看到，是2输入3输出的。从物理含义上来看，“位移$x$”代表了一维的运动，“倾角$theta_p$”代表了平衡车的姿势，“偏航角$delta$”代表了二维平面上的转向。

= 普通控制器设计

== *思路概述*

#t 我的想法是先实现普通的PID控制，在成功后，使用更高级的模糊控制以及神经网络控制提升控制效果，逐步迭代。

#t 由于系统有三个输出，在实现简单的PID控制时，要考虑对象的工艺特性，即三个变量之间的逻辑关系。根据框图看出，可把两个输入视作其共模信号及差模信号。航向角$delta$的变化是双轮差动的结果，因此$delta$仅受到差模信号影响，共模信号对转向的贡献为$0$；同理，倾角与位移仅仅受到共模信号的影响，差模信号不会对倾角和位移产生作用。因此，可以很方便地解耦：把$delta$单独拎出去控制，操纵变量就是差模信号（用一个单回路控制它即可）；倾角和位移则统一用共模信号控制。

#t 不难发现，位移和倾角都是由同一个操纵变量控制的！（2输入3输出问题本身就蕴含“至少有两个变量是被同一个操纵变量控制”这一点）因此，不可能使用两个单回路控制成功。分析对象特性即知，倾角不为$0$时，车体是歪斜的，$x$也不可能稳定下来，因此两者的控制含有隐藏的逻辑：需先控制$theta_p$，再稳住$x$；$x$是时间常数大的主变量，而$theta_p$是响应速度快的副变量。可以借鉴*串级控制*思想。

#t *注*：可能会觉得这不是典型的串级控制。主副变量一个是$x$，一个是$theta_p$，光看Simulink框图可能难以看出其主/副层次关系。但是观察原微分方程，可以这样想：$theta_p$的变化确定后，$x$的各阶导数被相应的解出来。因此，为了简洁，下文中使用串级控制的概念解释。

#t 至此，整个控制框架已经搭建，参见下图：
#purple_theorem([*Simulink*])[#image("./images/PID_structure.png")]

#t 接着具体分析三个控制器的特色：

+ 由于小车稳定下来以后，倾角一定是$0$，因此不管怎么样，控制倾角的控制器，都是一个PD控制器（不应该含有积分作用），故主回路控制器是PD控制器。

+ 由于差模信号非零时，一定会发生转向，因此到达稳态工作点时，差模信号必然为$0$，故，差分转向控制器也不用含积分项，同为PD控制器。

== *普通PID实现控制*

（在文件balance_car.slx中）调整参数（Simulink中的[P,I,D]值）：
- 主回路控制器=[0.01,0,0.2]
- 副回路控制器=[-280,-200,-120]
- 差分转向控制器=[1,0,0.8]

在三个被控变量$dot.double(delta),dot.double(x),dot.double(theta)_p$的处叠加上白噪音（作为理想模型之外的干扰），得到对比效果如下：

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 1pt,
    align: center+horizon,
    [无\ 噪 \ 音],[#image("./images/PID_result.png")],[#image("./images/PID_result2.png")],
    [有 \ 噪 \ 音],[#image("./images/PID_result2_noise.png")],[#image("./images/PID_result_noise.png")],
    ),
    caption: [PID控制效果，Baseline]
)

= 加入模糊控制

#t 由于模糊控制器本身相当于PD控制器，因此可以将回路中的两个PD控制器（主回路、差分转向控制器）都用模糊控制器取代，实现更加平滑的控制效果。

#t FIS使用*Mamdani*类型，下面展示本次实验使用的模糊控制器：

== *Fuzzification*

#t 模糊控制器的首个输入定义为位移误差偏移量$e$（即与目标稳态点差值），第二个输入为$dot(e)$，现在，将位移偏差，和速度偏差的论域按照如下定义，设计两者的各自的一组隶属度函数：

*位移偏移*

#t 主回路控制器中$e$的论域为$[-0.8,0.8]$，取5个分类，中间用三角形函数，左右用z形和s形函数：

#figure(
  image("./images/Delta_x.png",width: 60%),
  caption: [$Delta x$的模糊规则]
)

#t  差分转向控制器中$e$的论域为[-0.3 0.3]，其余同。

*误差导数*

#t 其论域为$[-0.5,0.5]$，中间用三角形函数，左右用z形和s形函数：
#figure(
  image("./images/dot_x.png",width: 60%),
  caption: [$Delta x$的模糊规则]
)

#t  差分转向控制器中$dot(e)$的论域为[-0.3 0.3]，其余同。

*操作变量*

#t 输出的$theta_p$角度设定值的论域为$[-0.2,0.2]$，取了7个分类，也均在中间用三角形函数，左右用z形和s形函数

#figure(
  image("./images/output.png",width: 60%),
  caption: [$U$的模糊规则]
)

#t  差分转向控制器中输出（$theta_p$设定值）的论域为[-0.2 0.2]，其余同。

== *Inference*

#t 在我的两个控制器的模糊推断操作中，使用隶属度函数的$max$来作为OR（或）操作，用$min$来作为AND（和）操作，如下图所示：

#figure(
  image("./images/inference.png",width:60%),
  caption: [模糊逻辑 推断规则]
)


== *Defuzzification*

#t 两个控制器均使用centroid方法（见上图），即解模糊化的方法是计算两个区域的面积再取重心的横坐标（参见后文效果图）

== *Rules*

#t 两个控制器均采用下面的模糊规则。该控制规则的设计思路仍是：“远则快速接近，近则微调”。以主回路控制器为例说明：对于位移和速度均有负偏差时（说明小车不仅位置正向超前，而且速度也是正向太快了），需要输出较大的负向角度，对于位移正向超前但速度为负向时，说明已经在返回的路上了，因此只用少量的负向角度，甚至偏正的角度，赶紧慢下来刹车就行。对于正反互换的情况完全同理，因此整张表呈现出中心对称的特点。


#align(center)[

  #tablex(


    fill: (x, y) => if  calc.min(x,y)<=1 and x+y>0 {aqua.lighten(90%)} else if (calc.rem(y+x, 2)==0) {aqua.lighten(70%)} else{white},

    columns: 7,
    map-hlines: h => (..h, stroke: luma(225) + 0.05em),
    map-vlines: v => (..v, stroke: luma(225) + 0.05em),
    align: center+horizon,
    inset: 10pt,
    [],[],colspanx(5)[$e$],
    rowspanx(6)[$dot(e)$],[],[NN],[N],[ZERO],[P],[PP],
    [NN],[FFF],[FFF],[FF],[F],[NONE],
    [N],[FFF],[FF],[F],[NONE],[Z],
    [ZERO],[FF],[F],[NONE],[Z],[ZZ],
    [P],[F],[NONE],[Z],[ZZ],[ZZZ],
    [PP],[NONE],[Z],[ZZ],[ZZZ],[ZZZ],
  )
]



== *Visualization*

#t 这是centroid解模糊化规则的可视化（以主回路控制器为例，转向控制器同）：

#figure(
  table(
    stroke: luma(220)+0.1em,
    columns: 2,
    gutter: 1em,
    inset: 0pt,
    [#image("./images/view_rules.png",height:22%)],[#image("./images/view_rules2.png",height:22%)],
  ),caption: [模糊规则的可视化]
)

#t 以下是通过控制曲面的方法可视化模糊控制器（以主回路控制器为例，转向控制器同）：


#figure(
  table(
    stroke: luma(220)+0.1em,
    columns: 2,
    gutter: 1em,
    inset: 0em,
    [#image("./images/surface.png",height:22%)],[#image("./images/surface2.png",height:22%)],
  ),caption: [控制曲面]
)


== *Simulink Model*

#purple_theorem([*Simulink*])[#image("./images/fuzzy_simulink.png")]

== *Result*

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 1pt,
    align: center+horizon,
    [无\ 噪 \ 音],[#image("./images/fuzzy_result.png")],[#image("./images/fuzzy_result2.png")],
    [有 \ 噪 \ 音],[#image("./images/fuzzy_result_noise.png")],[#image("./images/fuzzy_result2_noise.png")],
    ),
    caption: [模糊控制效果]
)


#t 仍然使用之前PID的那几个输入波形，尝试跟踪，发现明显效果好于普通PID，追踪十分平滑，没有振荡。

= 加入神经网络控制

== *神经网络 引入原因*

#t 尽管模糊控制取得了很好的效果，但是由于副回路还是用了PID控制器（积分项的作用难以用模糊控制器代替），因此还是有缺陷：#text(blue)[*当模型参数发生变化，或者数据不准确时，有可能失控*]，而果将副回路的控制器设计为神经网络PID控制器，让它自己找到合适的P、I、D参数，那么即使参数发生较大的变化，也能够取得良好的控制效果。

#t 在我的架构里，主控制器是负责“上层指挥”，即根据$x$的测量值决定$theta_p$的设定值，因此模型参数发生变化，主回路控制器也并不需要调整。但是，具体$theta_p$怎么达到其设定值，是副回路的工作，因此副回路控制器才是与实际参数相关的，这就是把副回路的控制器用神经网络PID代替的理由。

#t 总之，使用神经网络PID#text(blue)[*并不是因为它效果好*]（普通PID只要参数设置正确，效果和神经网络PID当然差不多！）而是因为它能够#text(blue)[*自己去找合适的三个参数*]，因此带来巨大的便捷，非常robust，模型#text(blue)[*参数大幅度变化*]，仍然能控制。

== *神经网络框架*

#t 下面，写了一个三层的BP神经网络，输出为P、I、D三个参数：

#t 输入为$mat(e(k), e(k-1),e(k-2),e(k-3))^top$，首先乘一个上权重矩阵$W$，得到$vectorarrow(text("net"))$：

$ mat(w_11 ,w_12, w_13, w_14;w_21, w_22, w_23, w_24;w_31, w_32, w_33, w_34) mat(e(k); e(k-1);e(k-2);e(k-3))=mat(text("net1");text("net3");text("net3"))=vectorarrow(text("net")) $

#t 接着经过sigmoid函数激活: $sigma(vectorarrow(text("net"))) = vectorarrow(text("hidden_output"))$

#t $vectorarrow(text("hidden_output"))$作为下一层（输出层）的输入，输出层首先用权重矩阵$K$与之相乘：

$ mat(k_11, k_12,k_13;k_21,k_22,k_23;k_31,k_32,k_33) mat(text("hidden_output1");text("hidden_output2");text("hidden_output3")) = mat(text("sum1");text("sum2");text("sum3")) $

最后激活一下，作为*归一化*后的$P,I,D$三个参数的输出：

$ mat(P;I;D) = mat(sigma(text("sum1"));sigma(text("sum2"));sigma(text("sum3"))) $

#t 最后分别乘上$K_p,K_i,K_d$作为最终的三个参数，输出给varying PID模块。

== *数学分析*

#t 下面推导反向传播公式，将系统抽象为$u -> text("system") -> y$，而$u$是根据PID规则给出的，而PID参数又是网络给出的，因此可以根据系统的输出$y$来反向传播，更新网络参数。首先取loss函数为$J = 1/2 e^2 = 1/2 (r-y)^2$，因此$ partialderivative(J,y) = e(k) $

#t 由于系统动力学模型未知（这也是智能控制的假设），可以用符号函数代替输入输出之间的偏导关系(这一项需要把观测到的两个时刻的$y$以及$u$都收集起来才能算)：$ partialderivative(y,u)approx sgn((y(k)-y(k-1))/(u(k)-u(k-1))) =sgn((y(k)-y(k-1))times (u(k)-u(k-1))) $

#t 接下来计算输入$u$与三个参数的关系（回想增量式PID即得）：
#t $ & partialderivative(u,P)= K_p (e(k)-e(k-1))\
& partialderivative(u,I)=K_i e(k)\

& partialderivative(u,D) = K_d (e(k)+e(k-2)-2 e(k-1)) $

#t 再根据sigmoid函数的导数，即得：$ & partialderivative(P,text("sum1")) = P(1-P) \ & partialderivative(I,text("sum2")) = I(1-I)\
& partialderivative(D,text("sum3")) = D(1-D)
$

#t 而剩下的就是常规的神经网络反向传播，

$ partialderivative(text("sum")i,k_(i j)) = sigma(text("net")_j)  quad i =1,2,3 $

最后，对于输出层的更新为：

$ & partialderivative(J,k_(1j)) = partialderivative(J,y) partialderivative(y,u) partialderivative(u,P) partialderivative(P,text("sum1")) partialderivative(text("sum1"),k_(1j)) \
& partialderivative(J,k_(2j)) = partialderivative(J,y) partialderivative(y,u) partialderivative(u,I) partialderivative(I,text("sum2")) partialderivative(text("sum2"),k_(2j)) \
& partialderivative(J,k_(3j)) = partialderivative(J,y) partialderivative(y,u) partialderivative(u,D) partialderivative(D,text("sum3")) partialderivative(text("sum3"),k_(3j))
$

对于隐藏层更加复杂，为：

$ partialderivative(J,w_(i j)) = partialderivative(J,y) partialderivative(y,u) (partialderivative(u,P) partialderivative(P,text("sum1")) partialderivative(text("sum1"),text("hid_output"i))+ partialderivative(u,I) partialderivative(I,text("sum2"))partialderivative(text("sum2"),text("hid_output"i))  + partialderivative(u,D) partialderivative(D,text("sum3"))partialderivative(text("sum3"),text("hid_output"i))) times \ partialderivative(text("hid_output"i),text("net"i)) partialderivative(text("net"i),w_(i j)) $

#t 其中，$partialderivative(text("sum1"),text("hid_output"i))$显然就是$K$矩阵中的元素；$partialderivative(text("hid_output"i),text("net"i))$这一项就是sigmoid求导，即$text("hid_output"i) (1-text("hid_output"i))$；$partialderivative(text("net"i),w_(i j))$这一项即$e(k-j+1)$，至此，反向传播的每一项都清楚了，最后更新的结果即为：

$ w_(i j ) <-  w_(i j) - alpha partialderivative(J,w_(i j)) quad quad  k_(i j ) <-  k_(i j) - alpha partialderivative(J,k_(i j)) $

#t 沿袭以上记号，具体代码在文件NN_controller.m中，用S-function实现。

== *Simulink Model*

#t （以下内容在balance_car_with_NN.slx文件）副回路控制器改用这个神经网络PID控制器，而主回路控制器和差分转向控制器仍然用普通PID。

#purple_theorem([*Simulink*])[#image("./images/NN_PID.png")]

#t 以下为副回路控制器用“神经网络PID”和“普通PID”效果的对比，运行new_param.m文件后，更改其中的$m$参数，即质量发生变化后，看看哪个控制器更robust。位移$x$的参考信号是正弦函数。

#t 以下为$theta_p$与其设定值的对比，在各个情况中，神经网络PID效果都很好，而普通PID随着参数$m$的变小，效果越来越差：


#block(
inset: 3pt,
fill: luma(240),
[```matlab
m => 变化 %% 以下曲线使用参数如下
r = 0.07/2; I = 0.5*m*r^2; M = 0.757-2*m; l = 0.5*0.8;
J_p = (1/12)*M*(0.3^2+0.08^2); d = 0.1612; J_delta = (1/12)*M*(0.0930^2+0.0530^2);
```],
)


#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [m],[神经网络PID],[普通PID],
    [0.2],[#image("./images/NN_theta.png",height:22%)],[#image("./images/pid_theta.png",height: 22%)],
),
)

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,

    [0.1],[#image("./images/NN_theta1.png",height:22%)],[#image("./images/pid_theta1.png",height:22%)],
    [0.04],[#image("./images/NN_theta2.png",height:22%)],[#image("./images/pid_theta2.png",height:22%)],
 ),
    // caption: [神经网络PID与普通PID对比控制效果]
)

#t 在$m$参数值不同的时候，可以在scope中观察神经网络输出向量，发现P/I/D在$m$不同时确实会自动变的不同。（可以点开balance_car_with_NN.slx文件中的scope，看神经网络输出的那个向量值）

#t 仔细观察$m=0.04$的情况如下，普通PID控不住$theta_p$，导致了$x$也发生灾难性的发散。

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [参数],[$x$控制效果],[$theta_p$控制效果],
    [m \ = \ 0.04 \ PID],[#image("./images/pid_theta_result2.png",height:22%)],[#image("./images/pid_theta2.png",height:22%)],
    ),
    caption: [m=0.04时的普通PID控制效果]
)
\

#t 但是神经网络PID却仍然可行。对比说明神经网络PID的确可以适应模型参数的变化：


#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 3,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [参数],[$x$控制效果],[$theta_p$控制效果],
    [m \ = \ 0.04 \ NN],[#image("./images/NN_theta_result2.png",height:22%)],[#image("./images/NN_theta2.png",height:22%)],

    ),
    caption: [m=0.04时的神经网络PID控制效果]
)


= 神经网络PID与模糊控制 综合

#t （以下内容在balance_with_fuzzy_and_NN.slx文件中）模糊控制能够平滑实现$x$的跟踪（输出比PID更优的$theta_p$），而神经网络PID则对参数的变化有更强的适应性，结合两者，得到最终的控制结构，至此，已经将所有的普通PID都替换掉了！

#purple_theorem([*Simulink*])[#image("./images/final_structure.png",width: 100%)]

#t 这个方案的优势在于，能够实现平滑阶跃跟踪，而且对于模型参数的变化robust，下面是实验，接连改动param.m文件中的质量$m$以及$l$两个参数，两个参数都变化较大的倍数，看看是否仍然能控制（在控制过程中在是哪个变量$dot.double(x),dot.double(delta),dot.double(theta_p)$处都叠加加上白噪音噪音，模拟真实情况）。

#block(
inset: 3pt,
fill: luma(240),
[```matlab
%% 以下曲线使用参数如下
m => 变动; l => 变动;
r = 0.0672/2; I = 0.5*m*r^2;
M = 0.757-2*m; J_p = (1/12)*M*(0.0903^2+0.0530^2);
d = 0.1612; J_delta = (1/12)*M*(0.0930^2+0.0530^2);
```],
)

#figure(
  tablex(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 4,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [m],[$l = 0.5  times 0.0903$],[$l = 0.5  times 0.2$],[$l=0.5 times 0.4$],
    [0.035],[#image("./images/original_param.png")],[#image("./images/l=0.1.png")],[#image("./images/l=0.2.png")],
    [0.05],[#image("./images/m=0.05.png")],[#image("./images/m=0.05_l=0.1.png")],[#image("./images/m=0.05_l=0.2.png")],
    [0.3],[#image("./images/m=0.3.png")],[#image("./images/m=0.3_l=0.1.png")],[#image("./images/m=0.3_l=0.2.png")],
 ),
    caption: [模糊+神经网络PID，参数大幅度变动也能控制]
)

#t 可见上图中，当$m$参数变化8倍以上，而$l$参数变化4倍以上时，仍然能够控制，并不需要重新修改任何控制器参数，这体现出神经网络PID的优越性。

== *与普通PID对比*

#t 可以使用balance_car.slx中的普通PID控制进行对照，在这两个参数变动时控制失败：

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 2,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [m=0.035 , l=0.5$times$0.4],[m=0.3,l=0.5$times$0.0903],
    [#image("./images/pid_fail_l=0.2.png",height: 25%)],
    [#image("./images/pid_fail_m=0.3.png",height: 25%)],
 ),
    caption: [普通PID，参数变动时控制失败]
)

== *与仅用模糊控制对比*

#t 神经网络PID+模糊控制器，与仅仅添加了模糊控制的模型对比，也是具有优越性：运行new_param.m后，运行balance_car_with_fuzzy_logic.slx文件，运行效果十分振荡，而运行balance_with_fuzzy_and_NN.slx中的神经网络PID+模糊的控制效果仍然不错：

#block(
inset: 3pt,
fill: luma(240),
[```matlab
%% 以下曲线使用参数如下
m = 0.022; r = 0.07/2; I = 0.5*m*r^2; M = 0.757-2*m; l = 0.5*0.8; J_p = (1/12)*M*(0.3^2+0.08^2); d = 0.1612; J_delta = (1/12)*M*(0.0930^2+0.0530^2);
```],
)

#figure(
  table(
    stroke: oklab(80.74%, 0.118, -0.087, 48.6%)+0.1em,
    columns: 2,
    gutter: 0em,
    inset: 2pt,
    align: center+horizon,
    [仅仅用模糊控制器],[模糊控制器+神经网络PID],
    [#image("./images/fuzzy_fail.png",height: 25%)],
    [#image("./images/final_ok.png",height: 25%)],
 ),
    caption: [仅用模糊控制，参数变动时控制失败]
)

= 总结

#t 本项目的对象是平衡车，2输入3输出，首先借鉴了串级控制的思路搭建控制框架，然后用普通PID进行初步控制，效果作为baseline。

#t 引入模糊控制改进控制效果，使得跟踪更加平滑，效果明显优于baseline。但是，模型参数变动后，仍然需要重新调参（因为副回路的普通PID控制器仍然依赖于模型参数信息），因此使用神经网络PID弥补这一点，使得控制更robust。

#t 对比“神经网络PID搭配两个普通PID”与“三个普通PID”之间的差异，发现在原参数下，两者效果接近（因为原本PID已经调到较优秀的参数了），但是当模型参数变动后，普通PID就失控了，然而神经网络PID仍然有较好的控制效果。

#t 最终，将两个模糊控制器与一个神经网络PID一道放入回路，完全取代了原本的PID，控制效果与robustness均优于普通PID。又和之前仅用模糊控制的方法对比，发现该最终方案更robust。

= 代码附件与运行说明

> 使用的Matlab版本为2023b

附件含有：\
> balance_car.slx 仿真模型，使用三个普通PID控制 \
> balance_car_with_fuzzy_logic.slx 仿真模型，使用两个模糊控制器+一个普通PID控制 \
> balance_car_with_NN.slx 仿真模型，使用一个神经网络PID控制器+两个普通PID控制 \
> balance_with_fuzzy_and_NN.slx 最终仿真模型，使用两个模糊控制器+一个神经网络PID控制 \

> parameter.m 模型参数（来源于真实数据）\
> new_parameter.m 人为改动后的参数（为了测试robustness）

> fuzzy_controller.fis 主回路模糊控制器 \
> fuzzy_delta.fis 差分转向模糊控制器 \
> NN_controller.m 神经网络PID的level2 S-function文件

#t 测试时，请先运行parameter.m文件或者new_parameter.m文件（使得workspace中有参数的数据）然后即可运行slx文件。


= 参考资料

#t 模型来自该处提供的实物数据：#text(blue)[#link("https://pan.baidu.com/e/1igIu6VU-7f1i702oJFCTrQ")]（但该资料的模型是线性化后的，我的项目中特意没有线性化，使用了精确的非线性方程）


#text(red.lighten(35%))[如有疑问，请访问 https://github.com/Maythics/Control_simulink.git]
