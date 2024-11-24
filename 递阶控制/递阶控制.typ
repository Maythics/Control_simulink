#import "newtemplate.typ": *
#import "@preview/dashing-dept-news:0.1.0": newsletter, article
#import "@preview/cuti:0.2.1": show-cn-fakebold
#show: show-cn-fakebold

// #cover(subtitle:"浙江大学控制科学与工程学院",
//         title:"企业认知实习报告",
//         name:"章翰宇",
//         teacher:"孟文超",
//         ID:"3220104133",
//         school:"浙江大学",
//         time:"2024.7",
//         figure: "./images/CSE_logo.png",
//       )


#show: apply-template.with(
  title:[*递阶控制*],
  right_header:"ZJU CSE",
  left_header:"Control Science and Engineering",
  author: "章翰宇",
  ID:"3220104133",
  logo_path:"./images/CSE_logo.png",
  abstract: [多机器人协作系统由三个机器人组成，分别为两个悬挂移动式多关节机器人（SMR1和SMR2）和一个地面移动式多关节机器人（GMR）。采用递阶控制的思想将多机器人协作系统进行分解，并画出该递阶控制系统的分级系统结构图],
  keywords:[智能控制，递阶控制],
  column: 1
  )

#set text(size: 12pt)

#figure(
  image("./images/robot_fig.png",width: 50%),
  caption:[多机器人协作系统]
)

= 递阶控制的分解层次：

== *组织级（顶层控制）*

+ 任务目标：从无序堆放的工件中拾取目标螺杆，将螺杆的两端装上合适的螺帽，然后放置到期望目标位置。
+ 输入：全局视觉数据（外部三维点云数据）以及任务规划信息。
+ 控制任务：定义整体任务和协作流程，例如：选择目标螺杆、规划装配路径、任务分配给各个机器人等。
+ 输出：任务分配、路径规划指令、目标位置等。

== *协调级（协作层）*

+ 协调任务：协调各个机器人（SMR1、SMR2、GMR）之间的合作，例如：确定每个机器人完成的具体任务（例如，哪个机器人负责拾取螺杆、哪个机器人负责装螺帽、哪个机器人负责放置螺杆）。
+ 输入：任务层传递的任务分配信息、机器人的位置、姿态、以及视觉和力传感器的数据。
+ 输出：各机器人之间的协作动作顺序，任务细节的执行指令。
+ 关键功能：通信与协作，防止机器人之间的冲突与资源争夺。

== *执行级（低层控制）*

+ 执行任务：各个机器人执行具体的机械操作，例如：抓取螺杆、装螺帽、精确放置螺杆到目标位置。
+ 输入：协作层传递的指令、机器人局部感知信息（深度相机、力矩传感器等）。
+ 控制任务：根据实时感知信息（如物体位置、力传感器数据等）进行精确的运动控制和抓取策略。
+ 输出：机器人具体的运动命令、抓取动作等。

感知与反馈（辅助）

+ 感知任务：获取工件的位置、姿态以及目标位置，通过各类传感器（外部三维点云相机、深度相机、机械臂力矩传感器等）对工作环境进行感知。
+ 输入：感知数据（点云、图像、力矩等），环境信息。
+ 输出：环境状态信息提供给上层控制（任务层、协调层），实现实时反馈和调整。

= 递阶控制系统的分级结构图：

#figure(
  image("./images/block.png", width: 100%),
  caption: "分层结构图"
)


- 组织级，任务层：负责任务的全局规划与协调，确保整体任务目标的完成。输入主要是来自外部三维相机的全局感知数据，输出任务分配给各机器人。

- 协调级，协作层：主要任务是协调各个机器人的动作，防止机器人之间发生冲突。该层负责信息的交换与协作指令的生成。

- 执行级，底层：每个机器人根据协调层的指令，执行低层控制，包括运动控制、抓取控制、装配等。这个层级需要依赖机器人的精确控制（如轨迹规划、抓取力度调整等）。

= 递阶控制的信息流动分析

- 从组织级到协调级：组织级是人类控制的接入口，负责将操作者的命令转化为顶级规划，然后由TAC分派下发给协调将全局任务规划和路径规划的结果传递给协调级，协调级根据这些信息协调各个机器人的具体任务。
- 从协调级到执行级：协调级将协作动作顺序和任务细节的执行指令传递给执行级，执行级根据这些指令执行具体的机械操作。
- 执行级，感知与反馈层：执行级在执行过程中，通过传感器收集局部感知信息，并将这些信息反馈。
#t 整个递阶控制系统的信息流动是一个各层次之间相互依赖的架构，通过信息的传递和反馈实现整个系统的协调运作。
