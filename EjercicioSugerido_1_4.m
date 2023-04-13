% Ejercicio Sugerido 1.4 
% Entrada Salida fase no mínima
clear;close all; clc
p=1;
NUM=[(1+p) -p];
DEN=[1 3 6];
Gs=tf(NUM, DEN)
% rlocus(Gs)
% polos de GS p1=-1.5 + 1.94i, p2=-1.5 - 1.94i ceros en Z1=1/2
GsLC=feedback(Gs,1)
% rlocus(GsLC)
% polos de GS p1=-3.62, p2=-1.38 ceros en Z1=1/2
T1=1/2;   %cero
T2=1/5;   %polo
alfa=T2/T1;
Kc=1;
C=tf(Kc*[1 1/T1], [1 1/T2]);
%rlocus(C*Gs)
step(Gs)