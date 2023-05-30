clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Tare Nº2
% ===================================================================

Sobrepaso=10/100;  % sobrepaso expresado en porcentaje
T=0.23;            % periodo [segundos]
tss= 4;            % tiempo de establecimiento [segundos] 

epsilon=(-log(Sobrepaso))/(sqrt(pi^2+log(Sobrepaso)^2));
w_o=4/(epsilon*tss);        % frecuencia natural [rad/seg]
w_d=w_o*sqrt(1-epsilon^2);  % frecuencia de amortiguamiento [rad/seg]
w_s=2*pi/T;   % frecuencia de muestreo [rad/seg]
Td=(2*pi)/w_d;
Tm=(2*pi)/w_s;
m=Td/Tm;    % cantidad de muestras

r=exp(-epsilon*w_o*Tm);  % modulo de z
% uso la función "fromRadians()" para convertirlo en ángulo
omega=fromRadians("degrees",w_d*T);    % argumento de z 

%polos deseados en sistema discretizado
pz1=complex(r*cos(omega),r*sin(omega)); % resultado un numero complejo 
pz2=conj(pz1);                          % resultado el conjudado de "pz1"

G=zpk([-10],[-2 -2],[5])
Gd=c2d(G,Tm,'zoh')
Gd_z=Gd.Z{1,1};
Gd_p1=Gd.P{1,1}(1,1);
Gd_p2=Gd.P{1,1}(2,1);
Gd_K=Gd.K;

%figure(1); hold on 
%plot(pz1,'sq','LineWidth',3,'MarkerSize',8,'Color','red') 
%plot(pz1','sq','LineWidth',3,'MarkerSize',8,'Color','blue')  
%rlocus(Gd) 

pc=0.6;   % defino al polo fijo en el origen

fi1=atand(imag(pz2)/(pc-real(pz2)));
theta1=atand(imag(pz2)/(real(pz2)-Gd_z));
fi2=180-atand(imag(pz2)/(Gd_p1-real(pz2))); % angulo del polo doble
theta2=mod(-theta1+2*fi2+fi1,180);                         % controlador (51º) 
d=imag(pz2)/tand(theta2);                     % distancia del cero 
cc=real(pz2)-d;                                % cero 

%Controlador de ganancia unitaria (auxliar)
Caux=zpk([cc],[pc],[1],Tm);
%Aplico condición de módulo para el cálculo de Kd
aux=Gd*Caux;
%kd=abs(evalfr(aux,pz2)); %valuo la funcion de transferencia en polo deseado

figure(1); hold on
plot(pz1','sq','LineWidth',3,'MarkerSize',8,'Color','red')  
rlocus(Gd*Caux)
kd=rlocfind(aux,pz1);   % valor definido graficamente
C_Ad=zpk([cc],[pc],[kd],Tm)

F=feedback(Gd*Caux*kd,1); 
F=zpk(F)
pole(F)
zero(F)
figure(2);step(F);
figure(3);hold on; pzmap(F)
t=0:Tm:20; % rampa tiempo 
figure(4), lsim(F,t) % simula resp rampa 
error_ss=1-dcgain(F)

