clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Tarea Nº3 
% ===============================================================
% CASO 2: Sistema no lineal de cuatro variables de estado 
% -------------------------------------------------------

% Datos:
% ------

% Polos:
p1=-1;p2=2;
% Ganancia:
K=5;
% Tiempo de Establecimiento (2%)
tss=3;

% Sistema:
G=zpk([],[p1 p2],K);  
%sisotool(G)
%C=zpk([-1],[],0.66165); % FT de Controlador
%F=feedback(G*C,1);
%step(F);

Kc=0.53046;    % ganancia 
a=3;           % cero del controlador con signo invertido 
M=Kc;          % ganancia rele = +-ganancia Kc   
T=K*Kc/100;        % la ganancia total 
lineal=0;      % simula no lineal 
sim('bang_bang_hist_DI_PD') 
figure(1)  % error
plot(tout,yout(:,1));legend('Error');title('Error')  
grid on 
figure(2)  % plano de fases
plot(yout(:,1),yout(:,3));legend('Plano de Fases');title('Plano de Fases')
xlabel('Error'); ylabel('Derivada del Error');
grid on 
figure(3)  % señal de control 
plot(tout,yout(:,2));legend('señal de control');title('señal de control')
grid on 



