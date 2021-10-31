close all;
clear;clc;
%% ����켣���ٶ�,��ֹ����
%% Alt3
load('.\Planning_LonSpeed\Speed_Planning_Global_Alt3.mat');
t_end=44;%�������ʱ��
%% HandlingCourse
% load('.\Planning_LonSpeed\Speed_Planning_Global_HandlingCourse.mat');
% t_end=146;%�������ʱ��

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        Vehicle Parameters for 2DOF and 7DOF                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7DOF&2DOFģ�Ͳ������ṹ��car,����ģ���õĲ���
car.m = 1600;%Mt; % car mass, kg
car.g = 9.8;%g; % gravity acceleration
car.iz = 2000;%Jz; % car moment of inertia about cg, kg m^2
car.lf = 1.016;%a;% distance from front tires to cg, m
car.lr = 1.524;%b; % distance from rear tires to cg, m
car.kyf = 92512;% single front tire cornering stiffness,N/rad 
car.kyr = 67728;% single rear tire cornering stiffness,N/rad
% % ·��Ħ������
car.mu = 1;% ·��Ħ������
% psid_bound=0.85*car.mu*car.g./x0.xd;% 
% beta_bound=atan(0.02*car.mu*car.g); 

car.jw = 1.0;%Jw; % rotor and tire moment of inertia, kg m^2
car.lw = 1.5;%c; % track width, m
car.reff = 0.285;%Rw(1); % tire effective radius, m

%% Initial state and control,��ʼ״̬
VehPos0 = [Tra(1,1);Tra(1,2);Tra(1,3)];
VehSt0 = [0;0;Tra(1,6)];

%% �����ó���ģ�Ͳ�����3DOF with Fiala tire model
g=car.g;
m=car.m;
Iz=car.iz;
lf=car.lf;
lr=car.lr;
l=lf+lr;
Kyf=car.kyf;
Kyr=car.kyr;
mu=car.mu;
FzF = m*g*lr/l;     % N
FzR = m*g*lf/l;     % N
FxR_max = mu*FzR;
FyF_max = mu*FzF;
VehPar=[g,m,Iz,lf,lr,l,Kyf,Kyr,mu,FzF,FzR,FxR_max]';

%% �����ٶȿ��Ʋ���
Ks=0.1;
Kv=-10;

%% ����Ԥ��ʱ����Ƿ�ʹ��Ԥ��
Ta=0.1;
Error_Mode=1;


%% ���з���
figure(1);plot(Tra(:,1),Tra(:,2),'r');hold on %�滮�켣
figure(3);plot(Tra(:,7),Tra(:,6),'r');hold on %�滮�켣

sim('Vehicle3DOFModelwithLonnLatController.slx');
figure(1);plot(ans.x,ans.y,'go-','MarkerSize',2);xlabel('X[m]');ylabel('Y[m]');hold on
figure(2);plot(ans.tout,ans.Delta,'go-','MarkerSize',2);xlabel('t[s]');ylabel('Delta[rad]');hold on;
figure(3);plot(ans.tout,ans.Vx,'go-','MarkerSize',2);xlabel('t[s]');ylabel('Vx[m/s]');hold on;