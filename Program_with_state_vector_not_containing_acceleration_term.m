%Program with state vector not containing acceleration term	
clc;
clear all;
close all;
int_dis=5000;
v=300;
theta1= pi/4;
g=9.8;
f=3;      % acceleration factor
ann=0.005; % noise angle normalization 
rnn=30;   % noise range normalization
 
% 1st trajectory
t= [0:0.5:50];
r1= (int_dis +v*t);
x1= r1.* cos(theta1);
y1= r1.* sin(theta1);
figure(1);
plot(x1,y1, '.-b');
hold on;
figure(2);
plot(x1,y1, 'b');
rmax=(int_dis+v*50);
xmax=rmax*cos(theta1);
ymax=rmax*sin(theta1);
hold on;
grid on;
 
% Adding noise to first trajectory
t= [0:0.5:49.5];
r1= (int_dis+v.*t);
nr1=rnn*randn(size(t));
rn1=r1+nr1;
nt1=ann*randn(size(t));
thetan1=theta1+nt1;
xn1= rn1.* cos(thetan1);
yn1= rn1.* sin(thetan1);
figure(2);
plot(xn1,yn1, '.-r');
figure(3);
plot(xn1,yn1, '.-r');
hold on;
 
 
%2nd trajectory
rr2=(v^2/(f*g));
phi1= atan(rr2/(rmax-int_dis));
phi2=atan(rr2/(rmax-int_dis));
phi=(phi1+phi2);
t2= (pi+phi)*rr2/(300);
t=[50:0.5:50+t2];
thetar2= linspace(3*pi/4,-theta1-phi,2*t2+1);
xc= xmax+rr2*cos(theta1);
yc= ymax-rr2*sin(theta1);
x2=rr2.*cos(thetar2)+xc;
y2=rr2.*sin(thetar2)+yc;
figure(1);
plot(x2,y2,'.-b');
grid on;
title('True trajectory');
figure(2);
plot(x2,y2,'b');
title('True trajectory & Noise');
r2=sqrt(x2.^2+y2.^2);
theta2= atan(y2./x2);
 
% Adding noise to second trajectory
nr2=rnn*randn(size(t));
rn2= r2+nr2;
nt2=ann*randn(size(t));
thetan2=theta2+nt2;
xn2= rn2.* cos(thetan2);
yn2= rn2.* sin(thetan2);
figure(2);
plot(xn2,yn2, '.-r');
figure(3);
plot(xn2,yn2, '.-r');
hold on;
 
%3rd trajectory
t= [50+t2:0.5:125];
a=(rmax-int_dis)*cos(pi/4-phi)+int_dis/sqrt(2);
b=(rmax-int_dis)*sin(pi/4-phi)+int_dis/sqrt(2);
thetar3= pi+pi/4-atan(2*rr2/(rmax-int_dis));
rr3= (v.*(t-(50+t2)));
x3=rr3.*cos(thetar3)+a;
y3=rr3.*sin(thetar3)+b;
figure(1);
plot(x3,y3, '.-b');
figure(2);
plot(x3,y3, 'b');
axis equal;
hold on;
r3= sqrt(x3.^2+y3.^2);
theta3= atan(y3./x3);
 

% Adding noise to third trajectory
t= [50+t2+0.5:0.5:125];
rr3= (v.*(t-(50+t2)));
x3=rr3.*cos(thetar3)+a;
y3=rr3.*sin(thetar3)+b;
nr3=rnn*randn(size(t));
r3= sqrt(x3.^2+y3.^2);
theta3= atan(y3./x3);
rn3=r3+nr3;
nt3=ann*randn(size(t));
thetan3=theta3+nt3;
xn3= rn3.* cos(thetan3);
yn3= rn3.* sin(thetan3);
figure(2);
plot(xn3,yn3, '.-r');
figure(3);
plot(xn3,yn3, '.-r');
xlabel('x axis');
ylabel('y axis');
hold on;
 
% Initialization of matrices
T=0.5;
xn= [xn1,xn2,xn3];
yn= [yn1,yn2,yn3];
rn= sqrt(xn.^2+yn.^2);
thetanx= atan(yn./xn);
xyn=[xn1,xn2,xn3;yn1,yn2,yn3];
thetan= [thetan1 thetan2 thetan3];
 
A=[1 0 T 0 ; 0 1 0 T ; 0 0 1 0; 0 0 0 1];
Q=170*[T^3/3 0 T^2/2 0;   0 T^3/3 0 T^2/2;   T^2/2 0 T 0;   0 T^2/2 0 T];
 H=[1 0 0 0 ;0 1 0 0 ];
I= eye(4);
k=3;
X1=zeros(4,250);
X2=zeros(4,250);
X1(1,1)=xn(1,1);
X1(1,2)=xn(1,2);
X1(2,1)=yn(1,1);
X1(2,2)=yn(1,2);
X1(3,1)=(xn(1,2)-xn(1,1))/T;
X1(4,1)=(yn(1,2)-yn(1,1))/T;
X1(3,2)=(xn(1,2)-xn(1,1))/T;
X1(4,2)=(yn(1,2)-yn(1,1))/T;
Rangef(1,1)= sqrt(X1(1,1)^2 + X1(2,1)^2 );
Azimuthf(1,1)= atan(X1(2,1)/X1(1,1));
Rangef(1,2)= sqrt(X1(1,2)^2 + X1(2,2)^2 );
Azimuthf(1,2)= atan(X1(2,2)/X1(1,2));
x=[x1 x2(1,2:2*t2+1) x3];
y=[y1 y2(1,2:2*t2+1) y3];
R= [(std(xn-x))^2 0; 0 (std(yn-y))^2];
P1(1:4,5:8)=[R(1,1) 0 R(1,1)/T 0 ;
               0 R(2,2) 0 R(2,2)/T ;
               R(1,1)/T 0 2*R(1,1)/T 0 ;
               0 R(2,2)/T 0 2*R(2,2)/T ;];
X2(1,1)=xn(1,1);
X2(1,2)=xn(1,2);
X2(2,1)=yn(1,1);
X2(2,2)=yn(1,2);
X2(3,1)=(xn(1,2)-xn(1,1))/T;
X2(4,1)=(yn(1,2)-yn(1,1))/T;
X2(3,2)=(xn(1,2)-xn(1,1))/T;
X2(4,2)=(yn(1,2)-yn(1,1))/T;
Rangep(1,1)= sqrt(X2(1,1)^2 + X2(2,1)^2 );
Azimuthp(1,1)= atan(X2(2,1)/X2(1,1));
Rangep(1,2)= sqrt(X2(1,2)^2 + X2(2,2)^2 );
Azimuthp(1,2)= atan(X2(2,2)/X2(1,2));
 
%Looping
while k<251
   X2(:,k)=A*X1(:,k-1);
    P2(1:4,4*(k-1)+1:4*k)=A*P1(1:4,4*(k-2)+1:4*(k-1))*A'+Q;
    K=((P2(1:4,(4*(k-1)+1):(4*k))*H'))/(H*P2(1:4,4*(k-1)+1:4*k)*H'+ R);
    X1(:,k)= X2(:,k)+ K*(xyn(:,k)-H*X2(:,k));
    P1(1:4,4*(k-1)+1:4*k)=(I-K*H)* P2(1:4,4*(k-1)+1:4*k);    
    Rangef(1,k)= sqrt(X1(1,k)^2 + X1(2,k)^2 );
    Azimuthf(1,k)= atan(X1(2,k)/X1(1,k));
    Rangep(1,k)= sqrt(X2(1,k)^2 + X2(2,k)^2 );
    Azimuthp(1,k)= atan(X2(2,k)/X2(1,k));
    k=k+1; 
end
 
figure(3); 
plot(X1(1,:), X1(2,:), '.-k');
title('measured, predicted & filter output');
axis equal;
hold on;
figure(3);
plot(X2(1,:), X2(2,:), '.-g');
axis equal;
grid on;
hold on;
figure(4);
plot(Rangef(1,:), '.-k');
hold on;
figure(4);
plot(Rangep(1,:), '.-g');
title('Range vs Time');
grid on;
hold on;
figure(5);
plot(Azimuthf(1,:), '.-k');
hold on;
figure(5);
plot(Azimuthp(1,:), '.-g');
grid on;
title('Azimuth vs Time');
hold on;
figure(4);
plot(rn,'.-r');
figure(5);
plot(thetanx,'.-r');
figure(1);
plot(X1(1,:), X1(2,:), '.k');
axis equal;
 vf=sqrt(X1(3,:).^2+X1(4,:).^2);
figure(6);
plot(vf, 'k');
grid on;
 rn=[rn1 rn2 rn3];
r=[r1 r2 r3];
rangestd=std(Rangef-r)
theta1= pi/4* ones(1,100);
theta=[theta1 theta2 theta3];
azimuthstd=std(Azimuthf-theta)
rangestd1=std(Rangef(1:100)-r1)
azimuthstd1=std(Azimuthf(1:100)-theta1)
rangestd2=std((Rangef(101:100+t2*2))-r2(1,1:2*t2))
azimuthstd2=std(Azimuthf(101:100+t2*2)-theta2(1,1:2*t2))
rangestd3=std(Rangef(100+2*t2+1:250)-r3)
azimuthstd3=std(Azimuthf(100+2*t2+1:250)-theta3)
 velocitystd=std(vf-300)
velocitystd1=std(vf(3:100)-300)
velocitystd2=std(vf(101:101+2*t2)-300)
velocitystd3=std(vf(102+2*t2:250)-300)
