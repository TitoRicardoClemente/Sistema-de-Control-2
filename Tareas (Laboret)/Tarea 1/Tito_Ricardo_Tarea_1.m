clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023

G=zpk([-10],[-2 -2],5);
Tm=0.23*10;
Gd=c2d(G,Tm,'zoh');
F=feedback(Gd,1) % sistema discreto realimentado
% Calculo de Polos y Ceros
pole(F)
zero(F)
%defino mi nueva función de transferencia
k=1.705;
p=[-0.0285  -11.8634];
z=[-0.0284];
Gn=zpk(z,p,k)
subplot(2,1,1);
pzmap(Gn,'g');title('G_n Ceros y Polos')
Gnd=c2d(Gn,Tm,'zoh');
subplot(2,1,2);
pzmap(Gnd,'r');title('G_nd Ceros y Polos')



