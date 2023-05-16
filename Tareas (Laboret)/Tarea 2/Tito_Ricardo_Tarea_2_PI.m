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
%{
figure(1); hold on 
plot(pz1,'sq','LineWidth',3,'MarkerSize',8,'Color','red') 
plot(pz1','sq','LineWidth',3,'MarkerSize',8,'Color','blue')  
rlocus(Gd) 
%}
fi1=180-atand(imag(pz2)/(1-real(pz2)));       % angulo del integrador
fixx=180-atand(imag(pz2)/(0.6313-real(pz2))); % angulo del polo doble
theta1=atand(imag(pz2)/(real(pz2)-0.003315)); % angulo del cero 
theta2=mod(-2*fixx+theta1-fi1,180);           % controlador (79º) 
d=imag(pz2)/tand(theta2);                     % distancia del cero 
c=real(pz2)-d;                                % cero 

%Controlador de ganancia unitaria (auxliar)
Caux=zpk([c],[1],[1],Tm);
%Aplico condición de módulo para el cálculo de Kd
aux=Gd*Caux
kd=abs(evalfr(aux,pz2)); %valuo la funcion de transferencia en polo deseado


CdPI=zpk([c],[1],[kd],Tm)
GdCd= Gd*CdPI

figure(1); hold on
plot(pz1','sq','LineWidth',3,'MarkerSize',8,'Color','red')  
rlocus(Gd*CdPI)

F=feedback(Gd*CdPI,1); zpk(F) ;
figure(2); step(F);
%figure(3);hold on; pzmap(F)
