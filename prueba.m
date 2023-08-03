clc
clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%%TIEMPO%%%%%%%%%%%%%%%%%%%%%%%%

tf = 30;
ts = 0.2;
t = 0:ts:tf;
N = length(t);

%%%%%%%%%%%%%%%%%%%%%%%%%PARÁMETROS DEL ROBOT%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scale = 3;
b = 0.065*scale;
l1 = 0.09*scale;
l2 = 0.15*scale;
l3 = 0.04*scale;

%%%%%%%%%%%%%%%%%%%%%CONDICIONES INICIALES%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q1 = zeros(1, N+1);
q2 = zeros(1, N+1);
q3 = zeros(1, N+1);
q4 = zeros(1, N+1);

q1(1) = 0*(pi/180);
q2(1) = 45*(pi/180);
q3(1) = 90*(pi/180);
q4(1) = -45*(pi/180);

%%%%%%%%%%%%%%%%%%%%PUNTO OPERACIONAL%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hx = zeros(1, N+1);
hy = zeros(1, N+1);
hz = zeros(1, N+1);

%%% CINEMÁTICA DIRECTA

hx(1) = l1*cos(q2(1))*cos(q1(1))+l2*cos(q2(1)+q3(1))*cos(q1(1))+l3*cos(q2(1)+q3(1)+q4(1))*cos(q1(1));
hy(1) = l1*cos(q2(1))*sin(q1(1))+l2*cos(q2(1)+q3(1))*sin(q1(1))+l3*cos(q2(1)+q3(1)+q4(1))*sin(q1(1));
hz(1) = b+l1*sin(q2(1))+l2*sin(q2(1)+q3(1))+l3*sin(q2(1)+q3(1)+q4(1));

%%%%%%%%%%%%%%%%%%%%%%%%%ACCIONES DE CONTROL%%%%%%%%%%%%%%%%%%%%%%%%%

q1pRef = 0.1*sin(0.1*t);
q2pRef = 0.1*cos(0.1*t);
q3pRef = 0.2*sin(0.1*t);
q4pRef = 0.3*sin(0.4*t);

%%%%%%%%%%%%%%%%%%%%%%%%%BUCLE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N
    
    q1(k+1) = q1(k)+ts*q1pRef(k);
    q2(k+1) = q2(k)+ts*q2pRef(k);
    q3(k+1) = q3(k)+ts*q3pRef(k);
    q4(k+1) = q4(k)+ts*q4pRef(k);
    
    %cinemática directa
    hx(k+1) = l1*cos(q2(k+1))*cos(q1(k+1))+l2*cos(q2(k+1)+q3(k+1))*cos(q1(k+1))+l3*cos(q2(k+1)+q3(k+1)+q4(k+1))*cos(q1(k+1));
    hy(k+1) = l1*cos(q2(k+1))*sin(q1(k+1))+l2*cos(q2(k+1)+q3(k+1))*sin(q1(k+1))+l3*cos(q2(k+1)+q3(k+1)+q4(k+1))*sin(q1(k+1));
    hz(k+1) = b+l1*sin(q2(k+1))+l2*sin(q2(k+1)+q3(k+1))+l3*sin(q2(k+1)+q3(k+1)+q4(k+1));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%SIMULADOR%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scene = figure;
sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);
set(scene,'Color','White');
set(gca,'FontWeight','bold');
axis equal;
axis([-2 2 -2 2 0 1]);
view([-25 30]);
grid on;
box on;
xlabel('x [m]'), ylabel('y [m]'),zlabel('z [m]');
camlight('rigthligth')

%%%%%%%%%%%%%%%%INICIAR SIMULACIÓN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%A) Mostrar robot

armRobot;
H1 = armPlot(0,0,0,q1(1),q2(1),q3(1),q4(1),scale);hold on;
%B) Gráficas
H2 = plot3(hx(1),hy(1),hz(1),'b','LineWidth',3);
step = 1;

for k = 1:step:N
    delete(H1);
    delete(H2);
    H1 = armPlot(0,0,0,q1(k),q2(k),q3(k),q4(k),scale);
    H2 = plot3(hx(1:k),hy(1:k),hz(1:k),'b','LineWidth',3);
    pause(ts);
end
