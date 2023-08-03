clc
clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%    TIEMPO  %%%%%%%%%%%%%%%%%%%
tf = 10;
ts = 0.1;
t = 0:ts:tf;
N = length(t);

%%%%%%%%%%%%%%%%%   PARAMETROS DEL ROBOT    %%%%%%%%%%%%%%%
scale = 3;
b = 0.065*scale;
l1 = 0.09*scale;
l2 = 0.15*scale;
l3 = 0.04*scale;

%%%%%%%%%%%%%%%%%   CONDICIONES INICIALES   %%%%%%%%%%%%%%%%
q1 = zeros(1,N+1);
q2 = zeros(1,N+1);
q3 = zeros(1,N+1);
q4 = zeros(1,N+1);

q1(1) = 45*(pi/180);
q2(1) = 60*(pi/180);
q3(1) = -45*(pi/180);
q4(1) = 0*(pi/180);

%%%%%%%%%%%%%%%%%%% PUNTO OPERACIONAL %%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);
hy = zeros(1,N+1);
hz = zeros(1,N+1);

%   CINEMÁTICA DIRECTA
hx(1) = l1*cos(q2(1))*cos(q1(1))+l2*cos(q2(1)+q3(1))*cos(q1(1))+l3*cos(q2(1)+q3(1)+q4(1))*cos(q1(1));
hy(1) = l1*cos(q2(1))*sin(q1(1))+l2*cos(q2(1)+q3(1))*sin(q1(1))+l3*cos(q2(1)+q3(1)+q4(1))*sin(q1(1));
hz(1) = b+l1*sin(q2(1))+l2*sin(q2(1)+q3(1))+l3*sin(q2(1)+q3(1)+q4(1));

%%%%%%%%%%%%%%%%%%%     POSICION DESEADA    %%%%%%%%%%%%%%%%
hxd = 0.5;
hyd = -0.3;
hzd = 0.5;

%%%%%%%%%%%%%%%%%%%%%%      ACCIONES DE CONTROL  %%%%%%%%%%%%%%%
q1pRef = zeros(1,N);
q2pRef = zeros(1,N);
q3pRef = zeros(1,N);
q4pRef = zeros(1,N);

%%%%%%%%%%%%%%%%%%%%%   ERRORES     %%%%%%%%%%%%%%%%%%%%
hxe = zeros(1,N);
hye = zeros(1,N);
hze = zeros(1,N);

%%%%%%%%%%%%%%%%%%%%    BUCLE   %%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N

    %%%%%%%%%%%%% CONTROLADOR %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % a) ERROR
    hxe(k) = hxd-hx(k);
    hye(k) = hyd-hy(k);
    hze(k) = hzd-hz(k);

    he = [hxe(k);hye(k);hze(k)];

    % b) Jacobiano
    J11 = - l2*cos(q2(k) + q3(k))*sin(q1(k)) - l1*cos(q2(k))*sin(q1(k)) - l3*cos(q2(k) + q3(k) + q4(k))*sin(q1(k));
    J12 = - l2*sin(q2(k) + q3(k))*cos(q1(k)) - l1*cos(q1(k))*sin(q2(k)) - l3*sin(q2(k) + q3(k) + q4(k))*cos(q1(k));
    J13 = - l2*sin(q2(k) + q3(k))*cos(q1(k)) - l3*sin(q2(k) + q3(k) + q4(k))*cos(q1(k));
    J14 = - l3*sin(q2(k) + q3(k) + q4(k))*cos(q1(k));

    J21 = l2*cos(q2(k) + q3(k))*cos(q1(k)) + l1*cos(q1(k))*cos(q2(k)) + l3*cos(q2(k) + q3(k) + q4(k))*cos(q1(k));
    J22 =  - l2*sin(q2(k) + q3(k))*sin(q1(k)) - l1*sin(q1(k))*sin(q2(k)) - l3*sin(q2(k) + q3(k) + q4(k))*sin(q1(k));
    J23 = - l2*sin(q2(k) + q3(k))*sin(q1(k)) - l3*sin(q2(k) + q3(k) + q4(k))*sin(q1(k));
    J24 = -l3*sin(q2(k) + q3(k) + q4(k))*sin(q1(k));

    J31 = 0;
    J32 = l2*cos(q2(k) + q3(k)) + l1*cos(q2(k)) + l3*cos(q2(k) + q3(k) + q4(k));
    J33 = l2*cos(q2(k) + q3(k)) + l3*cos(q2(k) + q3(k) + q4(k));
    J34 = l3*cos(q2(k) + q3(k) + q4(k)); 

    J = [J11 J12 J13 J14;...
         J21 J22 J23 J24;...
         J31 J32 J33 J34];

    % c) Parametros de control

    K = [1 0 0;...
         0 1 0;...
         0 0 1];

    % d) Ley de control
    qpRef = pinv(J)*K*he;

    % e) Separar las acciones de control

    q1pRef(k) = qpRef(1);
    q2pRef(k) = qpRef(2);
    q3pRef(k) = qpRef(3);
    q4pRef(k) = qpRef(4);

    % f) Aplicar las acciones de control

    q1(k+1) = q1(k) + ts*q1pRef(k);
    q2(k+1) = q2(k) + ts*q2pRef(k);
    q3(k+1) = q3(k) + ts*q3pRef(k);
    q4(k+1) = q4(k) + ts*q4pRef(k);

    % Cinematica directa
    hx(k+1) = l1*cos(q2(k+1))*cos(q1(k+1))+l2*cos(q2(k+1)+q3(k+1))*cos(q1(k+1))+l3*cos(q2(k+1)+q3(k+1)+q4(k+1))*cos(q1(k+1));
    hy(k+1) = l1*cos(q2(k+1))*sin(q1(k+1))+l2*cos(q2(k+1)+q3(k+1))*sin(q1(k+1))+l3*cos(q2(k+1)+q3(k+1)+q4(k+1))*sin(q1(k+1));
    hz(k+1) = b+l1*sin(q2(k+1))+l2*sin(q2(k+1)+q3(k+1))+l3*sin(q2(k+1)+q3(k+1)+q4(k+1));
end

% SIMULADOR 3D
scene = figure;
sizeScreen = get(0,'ScreenSize');
set(scene,'position', sizeScreen);
set(scene,'Color','white');
set(gca,'FontWeight','bold');
axis equal;
axis ([-2 2 -2 2 0 1]);
view([-25 30]);
grid on;
box on;
xlabel('x[m]'), ylabel('y[m]'), zlabel('z[m]');
camlight('rightlight')

%INICIAR SIMULACION
%a. MOSTRAR ROBOT
armRobot;
H1 = armPlot(0,0,0,q1(1),q2(1),q3(1),q4(1),scale);hold on;
%b. GRAFICAS
H2 = plot3(hx(1), hy(1), hz(1),'b','LineWidth', 3);
H3 = plot3(hxd(1), hyd(1), hzd(1),'ro','LineWidth', 3);

step = 1;

for k = 1:step:N
    delete(H1);
    delete(H2);
    H1= armPlot(0,0,0,q1(k),q2(k),q3(k),q4(k),scale);
    H2 = plot3(hx(1:k), hy(1:k), hz(1:k),'b','LineWidth', 3);
    pause(ts);
end

%GRAFICAS
%Errores
error = figure;
set(error,'position',sizeScreen);
subplot(311);
plot(t,hxe,'b','LineWidth',3),grid on,xlabel('Tiempo[s]'),ylabel('Error hx [m]');
subplot(312);
plot(t,hye,'k','LineWidth',3),grid on,xlabel('Tiempo[s]'),ylabel('Error hy [m]');
subplot(313);
plot(t,hze,'g','LineWidth',3),grid on,xlabel('Tiempo[s]'),ylabel('Error hz [m]');

%Acciones controles
control=figure;
set(control,'position',sizeScreen);
subplot(221);
plot(t,q1pRef,'b','LineWidth',3), grid on,xlabel('Tiempo [s]'), ylabel('Velocidad q1pRef [rad/seg]');
subplot(222);
plot(t,q2pRef,'r','LineWidth',3), grid on,xlabel('Tiempo [s]'), ylabel('Velocidad q2pRef [rad/seg]');
subplot(223);
plot(t,q3pRef,'k','LineWidth',3), grid on,xlabel('Tiempo [s]'), ylabel('Velocidad q3pRef [rad/seg]');
subplot(224);
plot(t,q4pRef,'g','LineWidth',3), grid on,xlabel('Tiempo [s]'), ylabel('Velocidad q4pRef [rad/seg]');








