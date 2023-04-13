clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº1 Representación de sistemas y control PID 
% ===================================================================
% Circuito RLC
% Ve: tensión de entrada [Volts]
% L: inductancia [Henrios]
% C: capacidad [Faradios]
% R: resistencia [Ohms]
% Vr: tensión de salida (en la resistencia) [Volts]
% i: corriente

% Variables de Estado de sistema eléctrico
% dx/dt = A x + b u
% y = (cT) * x

% Definicón de las Matrices y los valores de Cada Variable
% Variables
R= 5.6e3;  % [KOhms] 
L= 10e-6;  % [uHy]
C= 100e-9; % [nF]
% Matrices
A= [-R/L -1/L; 1/C 0];
B=[1/L; 0];
cT=[R 0];
% Autovalores:
Autovalores_A=eig(A);
% Polos
lambda_1=Autovalores_A(1);
lambda_2=Autovalores_A(2);
% constante de tiempo del polo más rápido
tr=log(0.95)/lambda_1;
% constante de tiempo del polo más lento
tl=log(0.05)/lambda_2;
% Tiempo de Integración:
t_I=0.5e-10; % dos veces menos que la constante de tiempo tr
% Tiempo de simulación
t_S=0.0028;  % dos veces mas que la constante de tiempo tl
% Pasos de la simulación
pasos=t_S/t_I;

% Matrices de los Datos
t=[pasos];  %tiempo
u=[pasos];  %entrada
il=[pasos]; %corriente
Vc=[pasos]; %capacitor

%Condiciones Iniciales
X=[0;0];    
y=[0];     

% Datos Onda Cuadrada de Entrada
toc=1e-3;   %tiempo de subida de onda cuadrada
Ve=12;      %Amplitud máxima de la onda cuadrada


for ii=0:1:pasos
    
    %tiempo de simulación
    t(ii+1)=ii*t_I;
    % Entrada del sistema
    u(ii+1)=Ve;
    % Prueba para subida o bajada de Onda Cuadrada
    if(t(ii+1)==toc)
       Ve=-1*Ve;
       toc=toc+1e-3;
    end 
    %variables de estados
    il(ii+1)=X(1);
    Vc(ii+1)=X(2);
    %Sistema
    X_p=A*X+B*u(ii+1);
    X=X+t_I*X_p;
    Y=cT*X;
end

figure(1);
subplot(3,1,1)
plot(t,u);title('Entrada U_t')
subplot(3,1,2)
plot(t,il);title('x_1 = Corriente I_L')
subplot(3,1,3)
plot(t,Vc);title('x_1 = Corriente V_C')
xlabel('Tiempo [s]');
