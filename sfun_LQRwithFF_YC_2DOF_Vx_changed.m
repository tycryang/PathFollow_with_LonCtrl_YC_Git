function [sys,x0,str,ts,simStateCompliance] = sfun_LQRwithFF_YC_2DOF_Vx_changed(t,x,u,flag,car)
% % 把计算部分放入LQR模块中，这样算法的独立性就会变更好
% % 把计算模块放入Ini模块中，因为Ini模块不可以增加输出，所以将Kc和Delta_ff_coeff定义为全局变量，实际运行时间跟放在最开头差不多
% % 把前馈部分也纳入模块中，因为前馈需要用到kc3，即LQR的解
%% flag调用函数
switch flag
    case 0
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes();
    case 3
        sys=mdlOutputs(t,x,u,car);
    case {1,2,4,9} % unused flags
        sys=[];
    otherwise % error handling
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes()

%% simulink默认需要的部分
sizes = simsizes;%不要动
% 改这里的参数
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
% 改上面的参数
sys = simsizes(sizes);%不要动

x0  = [];% initialize the initial conditions
str = [];% str is always an empty matrix
ts  = [0.05 0];% initialize the array of sample times
simStateCompliance = 'UnknownSimState';
end

function sys=mdlOutputs(t,x,u,car)
% 计算控制量
lat_err=[u(1);u(2);u(3);u(4)];
psid_ref=u(5);
Vx=u(6);
%% 系统方程离散化，求解LQR，得到Kc
% % 车辆参数与初始速度
m=car.m;
Iz=car.iz;
lf=car.lf;
lr=car.lr;
Kyf=car.kyf;
Kyr=car.kyr;

%% 基于偏差的连续状态方程,状态=[lateral_error;lateral_error_rate;heading_error;heading_error_rate] 
A=[0,1,0,0;
    0,-(2*Kyf+2*Kyr)/m/Vx,(2*Kyf+2*Kyr)/m,(-2*Kyf*lf+2*Kyr*lr)/m/Vx;
    0,0,0,1;
    0,-(2*Kyf*lf-2*Kyr*lr)/Iz/Vx,(2*Kyf*lf-2*Kyr*lr)/Iz,-(2*Kyf*lf^2+2*Kyr*lr^2)/Iz/Vx];
B=[0;2*Kyf/m;0;2*Kyf*lf/Iz];

% Q=[1 0 0 0;
%     0 0 0 0;
%     0 0 0.05 0;
%     0 0 0 0];% 状态权重
% Q=[1 0 0 0;
%     0 0 0 0;
%     0 0 0.5 0;
%     0 0 0 0];% 状态权重 %DLC
Q=[0.05 0 0 0;
    0 0 0 0;
    0 0 1 0;
    0 0 0 0];% 状态权重 %straightline with CC
R=1;% 输入权重
% % 状态方程离散化
Ts=0.05;
Ad=(eye(4)+A*Ts/2)/(eye(4)-A*Ts/2);
Bd=Ts*B;
% % 离散LQR的数值求解过程
P=Q;
diff=Inf;
i=1;
while(i<1000) && (diff>0.001)
    P_temp=Ad'*P*Ad-Ad'*P*Bd*(R+Bd'*P*Bd)^(-1)*Bd'*P*Ad+Q;% 黎卡提方程/Riccati 方程
    diff=max(max(P_temp-P));
    P=P_temp;
    i=i+1;
end
Kc=(R+Bd'*P*Bd)^(-1)*Bd'*P*Ad;
Delta_fb = -Kc*lat_err;

%% 前馈计算
L=lf+lr;
k3=Kc(3);
e2_ss_coeff=-lr+lf*m*Vx.^2/2./Kyr/L;
Kv=m/2/L*(lr/Kyf-lf/Kyr);
Delta_ff_coeff=(L+Kv*Vx.^2+k3*e2_ss_coeff)./Vx;
Delta_ff=Delta_ff_coeff*psid_ref;
%% 输出
sys = [Delta_fb;Delta_ff];
end

