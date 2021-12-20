 %% main.m
close all
clear all
clc

Vo = 9; % airspeed - [m/s] (Determina parte da tolerância do controle)
m = 1.43;
g = 9.81;
rhoHo = 1.225;
rhoH = 1.112;

gamma0 = 10*pi/180; %classico 0.1763
%gamma0 = 41.6168*pi/180; %Andre Bruno 0.8884 DLo = Cdo/Clo
sigma0 = 0*pi/180;
DL = tan(gamma0)*cos(sigma0)
%atemp = 2; %Atraso temporal do atuador
h0 = 5000;%5000%30000
psi0 = 360*(pi/180); %40*pi/180;

wn = 150;%frequência natural atuador
zm = 0.7;%amortecimento atuador

meanw1 = 7; %7;
varw1 = 0.5; %0.5;
seed1 = 41; %41;

meanw2 = 6; %6;
varw2 = 0.5; %0.5;
seed2 = 41; %41;

meanw3 = 5; %5;
varw3 = 0.5; %0.5;
seed3 = 41; %41;

meanw4 = 4; %4;
varw4 = 0.5; %0.5;
seed4 = 41; %41;

meanw5 = 3; %3;
varw5 = 0.5; %0.5;
seed5 = 41; %41;



raioref = 400;
href = 1000; %1000%2000
Ang = 180; %em graus
haprox = 255;%255%1000
hfinal = haprox -96;%haprox -96%500

fonts = 16;

x0 = 1000;
y0 = 1000;
xf = 500;
yf = 0;

% prompt1 = 'Posição inicial em X? ';
% prompt2 = 'Posição inicial em Y? ';
% prompt3 = 'Posição final em X? ';
% prompt4 = 'Posição final em Y? ';
% x0 = input(prompt1)
% y0 = input(prompt2)
% xf = input(prompt3)
% yf = input(prompt4)



options = simset('SrcWorkspace','current');

sim('controlevento',[],options)

x(end)
y(end)
pp = 1:length(wx);
hvar = (h(20)-h(10))/(tempo(20)-tempo(10))
vvar = (sqrt(x(100)^2+y(100)^2) - sqrt(x(99)^2+y(99)^2)/(tempo(100)-tempo(99)))


% figure
% plot(tempo,psi)
% grid on
% 
% figure
% set(gca, 'FontSize', fonts)
% grid on
% hold on
% plot(tempo,x,'Linewidth', 2)
% plot(tempo,y,'r','Linewidth', 2)
% plot(tempo,h,'g','Linewidth', 2)
% ylabel('Distância [m]')
% xlabel('Tempo [sec]')
% legend('Distância em X','Distância em Y','Distância em H')

figure
set(gca, 'FontSize', fonts)
hold on
plot3(x, y, h)
grid on
%title('trajetória em 3D usando o PID')
%title('LaicanSat-3 trajectory of descent')

ylabel('Position Y[m]')
xlabel('Position X[m]')
zlabel('Altitude[m]')
%legend(num2str(wx(1,1)),num2str(wx(1,2)),num2str(wx(1,3)),num2str(wx(1,4)),num2str(wx(1,5)),num2str(wy(1,1)),num2str(wy(1,2)),num2str(wy(1,3)),num2str(wy(1,4)),num2str(wy(1,5)))
plot3(y(1),x(1),h(1),'b*')
plot3(x(end),y(end),h(end),'r*',xf,yf,0,'g*')
axis([ xf-400 xf+400 yf-400 yf+400 0 h0+1000]);


th = 0:pi/50:2*pi;
xunit = raioref * cos(th) + xf;
yunit = raioref * sin(th) + yf;
hunit = zeros(length(xunit));
plot3(xunit, yunit,hunit,'g','Linewidth', 2);


figure
set(gca, 'FontSize', fonts);
cor1 =  1:280;
cor2 =  280:115000;
cor3 =  115000:length(tempo);


hold on


% plot3(x,y,h,'-.b','Linewidth', 1.4)
plot3(x(cor1),y(cor1),h(cor1)/1,'--b','Linewidth', 2)
plot3(x(cor2),y(cor2),h(cor2)/1,'-.r','Linewidth', 2)
plot3(x(cor3),y(cor3),h(cor3)/1,'b','Linewidth', 2)

grid on
%title('trajetória em 3D usando o PID')
%title('LaicanSat-3 trajectory of descent')
ylabel('Position Y[m]')
xlabel('Position X[m]')
zlabel('Altitude[m]')
plot3(y(1),x(1),h(1),'b*')
plot3(x(end),y(end),h(end),'r*',xf,yf,0,'b*')
%legend('Initialization','Loiter','Final Approach')


plot3(xunit, yunit,hunit,'g','Linewidth', 2);
%legend('Loiter','Final Approach')




figure
set(gca, 'FontSize', fonts)
hold on
plot(tempo,(trajref - psiout)*(180/pi))
%plot(pp*1.2481,psiout)
grid on
%title('Erro entre sinal de referência e o sinal de saída')
%title('Error between the reference signal and the output signal')
ylabel('Ref - \Psi [Degrees]')
xlabel('Time [sec]')




figure
%subplot(2,1,1)
set(gca, 'FontSize', fonts)
hold on
plot(tempo,trajref*(180/pi),'b')
%plot(pp*1.2481,trajref)
grid on
%title('Psi da trajetória de referência')
% title('Psi of the reference trajectory')
% ylabel('Degrees')
% xlabel('Time [sec]')

%subplot(2,1,2)
% set(gca, 'FontSize', 16)
% hold on
plot(tempo,psiout*(180/pi),'--r')
%plot(pp*1.2481,psiout)
%grid on
%title('Psi de saída após o controle PID')
%title('Psi output after PID control')
ylabel('\Psi [Degrees]')
xlabel('Time [sec]')
legend('\Psi of the reference trajectory','\Psi output after PID control')


figure
subplot(2,1,1)
set(gca, 'FontSize', fonts)
%title('Wind Wx')
hold on
grid on
plot(pp,wx)
ylabel('Velocity [m/sec]')
xlabel('Time [sec]')

subplot(2,1,2)
set(gca, 'FontSize', fonts)
%title('Wind Wy')
hold on
grid on
plot(pp,wy)
ylabel('Velocity [m/sec]')
xlabel('Time [sec]')

figure
set(gca, 'FontSize', fonts)
hold on
plot(tempo,sigmaout*(180/pi))
%plot(pp*1.2481,sigmaout)
grid on
%title('Control Actuator Signal')
ylabel('Actuator Angle [Degrees]')
xlabel('Time [sec]')
axis([0 3250 -25 25]);
