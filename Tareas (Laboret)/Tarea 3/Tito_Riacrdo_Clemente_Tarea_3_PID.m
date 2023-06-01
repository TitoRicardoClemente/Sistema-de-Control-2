clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Tare Nº2
% ===================================================================

Sobrepaso=0;  % sobrepaso expresado en porcentaje
tss= 4;       % tiempo de establecimiento [segundos] 
c=[];
p1=-1;
p2=2;
K=5;
T=0.23;
G=zpk([],[p1 p2],K);
%sisotool(G)
C=zpk([-1.333 -1.333],[0],1.5732e05);

Kp=(C.K)*(C.Z{1,1}(1,1))*2;
Kd=(C.K)*(C.Z{1,1}(1,1));
Ki=(C.Z{1,1}(1,1))*(C.K)^2;

