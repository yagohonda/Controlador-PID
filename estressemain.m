close all
clear all
clc

Vo = 9; % airspeed - [m/s] (Determina parte da tolerância do controle)
m = 1.43;
g = 9.81;
rhoHo = 1.225;
rhoH = 1.112;

gamma0 = 10*pi/180;
sigma0 = 0*pi/180;
DL = tan(gamma0)*cos(sigma0);
%atemp = 2; %Atraso temporal do atuador
h0 = 5000;
psi0 = 360*(pi/180); %40*pi/180;

wn = 1;%frequência natural atuador
zm = 0.7;%amortecimento atuador

meanw1 = 7; %7;
varw1 = 10; %10;
seed1 = 41; %41;

meanw2 = 6; %6;
varw2 = 4; %4;
seed2 = 41; %41;

meanw3 = 5; %5;
varw3 = 0.5; %0.5;
seed3 = 41; %41;

meanw4 = 4; %4;
varw4 = 0.2; %0.2;
seed4 = 41; %41;

meanw5 = 3; %3;
varw5 = 3; %3;
seed5 = 41; %41;


raioref = 400;
href = 800;

figure
numb = 200;

for i=1:numb

% meanwx = 0;
% meanwy = 0;
% varwx = 0;
% varwy = 0;
% 
x0 = 1000+i*rand;
y0 = 1000+i*rand;
xf = 0;
yf = 0;

x00(i) = x0;
y00(i) = y0;

% prompt1 = 'Posição inicial em X? ';
% prompt2 = 'Posição inicial em Y? ';
% prompt3 = 'Posição final em X? ';
% prompt4 = 'Posição final em Y? ';
% x0 = input(prompt1)
% y0 = input(prompt2)
% xf = input(prompt3)
% yf = input(prompt4)

xfef(i)= xf;
yfef(i)= yf;

options = simset('SrcWorkspace','current');

sim('controlevento',[],options)



% figure
% set(gca, 'FontSize', 16)
% hold on
% plot3(x, y, h)
% grid on
% title('trajetória em 3D usando o PID')
% 
% ylabel('Posição Y[m]')
% xlabel('Posição X[m]')
% zlabel('Altitude[m]')
% %legend(num2str(wx(1,1)),num2str(wx(1,2)),num2str(wx(1,3)),num2str(wx(1,4)),num2str(wx(1,5)),num2str(wy(1,1)),num2str(wy(1,2)),num2str(wy(1,3)),num2str(wy(1,4)),num2str(wy(1,5)))
% plot3(y(1),x(1),h(1),'b*')
% plot3(x(end),y(end),h(end),'r*')
% axis ([-100 1100 -100 1200 0 11000]);


set(gca, 'FontSize', 32)
hold on
%plot3(x, y, h)
grid on
%title('Final position for each interaction')
%ylabel('Posição Y[m]')
%xlabel('Posição X[m]')
%zlabel('Altitude[m]')
%plot3(y(1),x(1),h(1),'b*')
plot3(x(end),y(end),h(end),'r*')
ylabel('Position Y[m]')
xlabel('Position X[m]')
axis ([-200 200 -200 200 0 100]);

xerror(i) = x(end);
yerror(i) = y(end);

end
plot3(xf,yf,0,'b+')
th = 0:pi/50:2*pi;
xunit = 200 * cos(th) + xf;
yunit = 200 * sin(th) + yf;
hunit = zeros(length(xunit));
plot3(xunit, yunit,hunit,'g','Linewidth', 1.4);
[v1,v2] = meshgrid(-200:50:200,-200:50:200);
v11 = 0:50:400;
v22 = 0:50:400;
v3 = v1*v2;
v4 = v1*v2;
quiver(v1,v2,v3,v4);

% %estatisticas x
% figure
% hist(xerror,50)
% x_mean = mean(xerror)
% x_std = std(xerror)
% x_median = median(xerror)
% 
% %estatisticas y
% figure
% hist(yerror,50)
% y_mean = mean(yerror)
% y_std = std(yerror)
% y_median = median(yerror)

MeanX = mean(x00)
MeanY = mean(y00)

CovX = cov(x00)
CovY = cov(y00)

%estatisticas gerais
% tot1(:,1) = xerror;
% tot1(:,2) = yerror;

for i=1:numb
tot(i) = (xerror(i)^2+ yerror(i)^2)^(1/2);
end

error = 0:i-1;

stot = sort(tot);
pertot = linspace(1/numb,1,numb);

figure
plot(stot,pertot);
set(gca, 'FontSize', 32);
grid on
hold on
%title('CDF Diagram')
ylabel('Percent of samples')
xlabel('Error Module [m]')


figure
set(gca, 'FontSize', 16);
grid on
histfit(tot)
title('Monte Carlo Diagram')
ylabel('Number of samples')
xlabel('Error Module')

figure
set(gca, 'FontSize', 16);
hist(tot)
grid on
hold on
x_y_mean = mean(tot)
x_y_std = std(tot)
x_y_median = median(tot)
title('Monte Carlo Diagram')
ylabel('Number of samples')
xlabel('Error Module')

pp = 1:length(wx);

figure
set(gca, 'FontSize', 16)
hold on
plot3(x, y, h)
grid on
title('trajetória em 3D')

ylabel('Posição Y[m]')
xlabel('Posição X[m]')
zlabel('Altitude[m]')
%legend(num2str(wx(1,1)),num2str(wx(1,2)),num2str(wx(1,3)),num2str(wx(1,4)),num2str(wx(1,5)),num2str(wy(1,1)),num2str(wy(1,2)),num2str(wy(1,3)),num2str(wy(1,4)),num2str(wy(1,5)))
%plot3(y(1),x(1),h(1),'b*')
plot3(x(end),y(end),h(end),'r*')
axis ([-300 1100 -300 1200 0 11000]);

figure
set(gca, 'FontSize', 16);
hold on
plot(error,xerror,'+r')
plot(error,xfef,'Linewidth', 3)
grid on
title('gráfico dos erros em X')

ylabel('Posição X[m]')
xlabel('Número da interação')

figure
set(gca, 'FontSize', 16);
hold on
plot(error,yerror,'+r')
plot(error,yfef,'Linewidth', 3)
grid on
title('gráfico dos erros em Y')

ylabel('Posição Y[m]')
xlabel('Número da interação')


figure
set(gca, 'FontSize', 16);
subplot(2,1,1)
set(gca, 'FontSize', 16)
title('Vento Wx')
hold on
grid on
plot(pp,wx)

subplot(2,1,2)
set(gca, 'FontSize', 16)
title('Vento Wy')
hold on
grid on
plot(pp,wy)
