clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº1 Representación de sistemas y control PID 
% ===================================================================
% Circuito RLC - CASO 2
% Simulación
t_S=1e-7;                   %tiempo de simulación
tF=0.04;                    %tiempo final de simulación
u=12;                       %entrada 12 [V]
TL=0;                    %Torque inicial [Nm] 
jj=0;                       %indice
X=-[0; 0; 0];           % Vector de Omega, Wr y ia
ii=0;                % indice     
 
for t=0:t_S:tF 
    ii=ii+1; 
    X=modmotor2(t_S, X, u, TL); 
    x1(ii)=X(1);                  % Omega
    x3(ii)=X(3);                  % ia
    ent(ii)=u;
end


t=0:t_S:tF; 
subplot(3,1,1);hold on;
plot(t,x1,'r');title('Salida y, \omega_t'); 
subplot(3,1,2);hold on; 
plot(t,x3,'b');title('Corriente I_a'); 
subplot(3,1,3);hold on; 
plot(t,ent,'g');title('Entrada V_a'); 
xlabel('Tiempo [Seg.]'); 

% Función del Motor
function [X]=modmotor2(t_S, xant, accion, torque) 
Laa=366e-6; 
J=5e-9; 
Ra=55.6; 
B=0; 
Ki=6.49e-3; 
Km=6.53e-3;
TL=torque; 
Va=accion; 
h=1e-7; 
omega=xant(1); 
wp=xant(2); 
ia=xant(3);
for ii=1:t_S/h 
    wpp =(-wp*(Ra*J+Laa*B)-omega*(Ra*B+Ki*Km)+Va*Ki)/(J*Laa); 
    iap=(-Ra*ia-Km*omega+Va)/Laa;
    wp=wp+h*wpp; 
    wp=wp-(TL/J); 
    ia=ia+h*iap;
    omega=omega+h*wp;
end 
X=[omega,wp,ia]; 
end