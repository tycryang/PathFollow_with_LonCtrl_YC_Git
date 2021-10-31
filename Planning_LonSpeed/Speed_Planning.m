clear;clc;
close all;
% ���ݹ켣x,y������·������s������
% data=importdata('PathXY_Alt3.xlsx');% Alt3
% save Path_Alt3_Trajectory.mat data;
% load('Path_Alt3_Trajectory.mat');
% data=importdata('PathXY_HandleCourse.xlsx');% handling course
% save PathXY_HandleCourse.mat data;
load('PathXY_HandleCourse.mat');
x0=data(:,1);y0=data(:,2);

%% ֱ�߶�ȡ��̫ϡ�����������ٶȹ滮����˲�ֵ
ds0=5;% ����ԭʼ���ݼ��ѡȡֱ�߶�ȡ����,�Դ���ԭʼ���
x(1)=x0(1);y(1)=y0(1);
p=2;j=2;
while j<size(data,1)
    dist=norm([x0(j),y0(j)]-[x(p-1),y(p-1)]);
    if dist<=ds0
        x(p)=x0(j);y(p)=y0(j);
        p=p+1;
        j=j+1;
    else
        x(p)=x(p-1)+5/dist*(x0(j)-x(p-1));
        y(p)=y(p-1)+5/dist*(y0(j)-y(p-1));
        p=p+1;
    end
end
s(1)=0;
for i=2:length(x)
    s(i)=s(i-1)+sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2);
end

% ���ʺͷ����
N=length(s);
k=zeros(N,1);
psi=zeros(N,1);
for i=2:N-1
    x_temp=x(i-1:i+1);
    y_temp=y(i-1:i+1);
    [k(i),psi(i)] = PJcurvature(x_temp,y_temp);
end
k(1)=k(2);k(end)=k(end-1);
psi(1)=psi(2);psi(end)=psi(end-1);

% k=LineCurvature2D([x',y']);
k=smooth(k,10);%������ǽ�������һ��ƽ���Ƿ����ã�

%% ��ͼ
% figure(1);
% plot(x,y,'ro');xlabel('x(m)');ylabel('y(m)');hold on
% plot(x,y,'bs');
% figure(2);
% plot(s,y,'bo-','MarkerSize',2);xlabel('s(m)');ylabel('y(m)');
% figure(3);
% plot(s,k,'ro-','MarkerSize',2);xlabel('s(m)');ylabel('kappa(1/m)');hold on

%% 
% �ٶȹ滮
% Լ��������
% 1. ·������Լ����
% 2. �ٶ���������Ϊ144km/h
% 3. �����������ļ��ٶ���ֵΪ abs(ax)<=3;abs(ay<=3);
% 4. ��ʻԱ������� ����trailing braking
% 5. �������ٶ����� [0,5;10,4;20,3;30,1;40,0.5]',(���٣����ٶ�)��
% 6. �������� 10m/s^2;
% 7. ���ٶ�72km/h
Vmax=120/3.6;
V0=80/3.6;
Accmax=3;
Decmax=3;
Aymax=3;
Aeng=[0,5;10,4;20,3;30,1;40,0.5];
Abrk=10;
% ��������Լ��1�������ٶ�3Լ��������õĳ����ٶȣ���������߳�������2,����ٶ�
Vx=sqrt(Aymax./abs(k));
Vx1=Vx;
Vx1(Vx1>Vmax)=Vmax;
Vx1(1)=min([Vx1(1),V0]);
ay1=Vx1.^2.*k;
figure(10);
plot(s,Vx1*3.6,'ro-','MarkerSize',2);hold on
figure(11);
plot(s,ay1,'ro-','MarkerSize',2);hold on

% ���Ǽ���Ԥ��3����ʻ����4�ͼ�������5����
Vx2=Vx1;
Len=length(s);
acc=zeros(Len,1);
for i=2:Len
    if Accmax^2-(Vx2(i-1).^2*abs(k(i)))^2<0
        acc(i-1)=0;
    else
        acc(i-1)=min([Accmax,sqrt(Accmax^2-(Vx2(i-1).^2*abs(k(i)))^2),interp1(Aeng(:,1),Aeng(:,2),Vx1(i-1))]);
    end
    Vx2(i)=sqrt(Vx2(i-1)^2+2*acc(i-1)*(s(i)-s(i-1)));
    Vx2(i)=min(Vx2(i),Vx1(i));
end
ay2=Vx2.^2.*k;

figure(10);
plot(s,Vx2*3.6,'g');
figure(11);
plot(s,ay2,'go-','MarkerSize',2);hold on
figure(12);
plot(s,acc,'go-','MarkerSize',2);hold on

% ���Ǽ���Ԥ��3����ʻ����4�ͼ�������6����
Vx3=Vx2;
dec=zeros(Len,1);
for i=Len:-1:2
    if Decmax^2-(Vx3(i).^2*abs(k(i)))^2<0
        dec(i)=0;
    else
        dec(i)=min([Decmax,sqrt(Decmax^2-(Vx3(i).^2*abs(k(i))).^2),Abrk]);
    end
    Vx3(i-1)=sqrt(Vx3(i)^2+2*dec(i)*(s(i)-s(i-1)));
    Vx3(i-1)=min(Vx3(i-1),Vx2(i-1));
end
ay3=Vx3.^2.*k;
figure(10);title('Speed');
plot(s,Vx3*3.6,'bo','MarkerSize',2);
figure(11);title('Ay');
plot(s,ay3,'bo-','MarkerSize',2);hold on
figure(12);title('Ax');
plot(s,-dec,'bo-','MarkerSize',2);hold on

% % ���ˣ�����������ٶȵĹ滮��Vx3��Ϊ���չ滮���ٶ� % %
% ��������켣�������Ҫ�������������ʱ��
t(1)=0;% ��ʼ�㶨��Ϊ0ʱ��
for i=2:Len
    acc0dec(i-1)=(Vx3(i)^2-Vx3(i-1)^2)/(2*(s(i)-s(i-1)));% ����������ļ��ٶ�
    t(i)=t(i-1)+(2*(s(i)-s(i-1)))/(Vx3(i)+Vx3(i-1)); % ���չ滮�ٶȾ��������ʱ��
end
figure(13);
plot(s(1:end-1),acc0dec,'bo-','MarkerSize',2);hold on
title('Ax_{total}');
figure(14);
plot(s,t,'bo-','MarkerSize',2);hold on
title('Time');

Tra=[x',y',psi,k,s',Vx3,t'];
% save Speed_Planning_Global_Alt3.mat Tra
save Speed_Planning_Global_HandlingCourse.mat Tra
