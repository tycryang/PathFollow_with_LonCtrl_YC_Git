自我介绍：车辆工程专业毕业，主要方向是车辆动力学仿真与控制，操纵稳定性范畴，对操纵动力学、悬架K&C特性、轮胎F&M特性比较了解吧，毕业后在主机厂做性能集成，感觉部门实力太弱，想换方向，思前想后想去觉得不如往风口上靠——做自动驾驶吧，同时为了发挥自己的优势，决定转行方向是自动驾驶控制算法工程师，所以便有了这么一个项目。
项目介绍：在上一个项目定速实现侧向轨迹跟随的基础上，增加车辆的纵向自由度，同时实现侧向和纵向的跟随。
轨迹的来源是CarSim，速度规划是单独做的，放在文件夹Planning_LonSpeed，程序里面说明比较清楚就不在此赘述。
侧向控制算法只是主要还是用的LQR，偏差计算有两种，基于距离最近点（投影点）和最近点增加预瞄。
纵向控制算法主要是位置和速度的双环PID控制，偏差计算是基于相对时间，规划这一时刻到某个点、什么速度，当前在哪个点、什么速度。
车辆动力学模型用的三自由度模型，轮胎模型是Fiala轮胎模型，考虑了摩擦圆的约束。

整个模型有一个小问题，方向角限定的范围是(-pi,pi]，超出会从一个极端直接穿越到另一个极端，目标和状态不同步时，似乎会有一个比较大的偏差。
以上。
