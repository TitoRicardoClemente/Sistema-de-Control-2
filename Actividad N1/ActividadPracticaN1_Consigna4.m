clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº1 Representación de sistemas y control PID 
% ===================================================================
% Circuito RLC - CASO 3
% Ve: tensión de entrada [Volts]
% L: inductancia [Henrios]
% C: capacidad [Faradios]
% R: resistencia [Ohms]
% Vr: tensión de salida (en la resistencia) [Volts]
% i: corriente

%Cargar Datos
datos_ex= xlsread('Curvas_Medidas_RLC.xls','Hoja1');
% guardo los datos del excel en una variable 
tiempo = xlsread('Curvas_Medidas_RLC.xls','Hoja1','A:A');
corriente_L=xlsread('Curvas_Medidas_RLC.xls','Hoja1','B:B');
tension_C=xlsread('Curvas_Medidas_RLC.xls','Hoja1','C:C');

% Definicón de las Matrices y los valores de Cada Variable
% Variables
C= 54.7e-3; % [F]
L= 8.2e-3; % [Hy]
R= 3.8e3;  % [Ohms] 


% Matrices
A= [-R/L -1/L; 1/C 0];
B=[1/L; 0];
cT=[R 0];

% Tiempo de Integración:
t_I=1e-4; % dos veces menos que la constante de tiempo tr
% Tiempo de simulación
t_S=0.1;  % dos veces mas que la constante de tiempo tl
% Pasos de la simulación
pasos=t_S/t_I;
t_inicial=0.05;

% Tomo solo valores mayores a 0,05 segundos

[~,lugar]=min(abs(t_inicial-tiempo));
il=[];
for ii=0:1:(length(tiempo)-1)
    
    if tiempo(ii+1)<=tiempo(lugar)
        il(ii+1)=0;
    else
        il(ii+1)=corriente_L(ii+1);
    end
end

% Matrices de los Datos
t=[pasos];  %tiempo
u=[pasos];  %entrada
corr_L=[pasos]; %corriente

%Condiciones Iniciales
X=[0;0];    
y=[0];     

% Datos Onda Cuadrada de Entrada
toc=50e-3;   %tiempo de subida de onda cuadrada
Ve=-1;       %Amplitud máxima de la onda cuadrada


for ii=0:1:pasos
    % Entrada del sistema
    u(ii+1)=Ve;
    % Prueba para subida o bajada de Onda Cuadrada
    if(tiempo(ii+1)<=toc)
        corr_L(ii+1)=0;
    else
        %variables de estados
        corr_L(ii+1)=X(1);
        %Sistema
        X_p=A*X+B*u(ii+1);
        X=X+t_I*X_p;
        Y=cT*X;
    end 
    
end

figure(1);
% Sistema Original por Datos 
plot(tiempo,il,'red'); hold on
% Sistema Aproximado. 
plot(tiempo,corr_L,'blue');
title('Corriente L / Curva Aproximada')
xlabel('Tiempo [segundos]')
ylabel('Amplitud Corriente L')