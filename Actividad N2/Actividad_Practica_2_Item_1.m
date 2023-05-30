clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº2 Diseño de controladores en variables 
% de estado en tiempo continuo
% ===================================================================

% Constantes del sistema
Laa=5e-3; J=0.004; Ra=0.2; 
Bm=0.005; Ki=6.5e-5; Km=0.055;

% Matrices
A=[-Ra/Laa -Km/Laa 0 ;
    Ki/J   -Bm/J   0 ;
    0       1      0];
B=[1/Laa ;
   0     ;
   0    ];
C=[0 0 1];
D=0;
% Calculos los autovalores (son tres)
autovalores_A=eig(A);
sigma1=autovalores_A(1);   % sigma1=0
sigma2=autovalores_A(2);   % sigma2=-1.2546
sigma3=autovalores_A(3);   % sigma3=-39.9954

% Tomo el autovalor con la dinámica más rápida y el más lento

% t_I=log(0.95)/sigma3;    % tr=0.00128
h=0.006;                  % defino un valor dos veces más chico
% t_S=log(0.05)/sigma2;    % t_s=2.3878
t_S=50;                     % defino un valor dos veces más grande
paso=round(t_S/h);         % 6666,7

% Construcción del sistema ampliado 
Aa=[A zeros(3,1);-C 0]; 
Ba=[B;0]; 
Ma=[Ba Aa*Ba Aa^2*Ba Aa^3*Ba];%Matriz Controlabilidad
rango_Ma=rank(Ma);% el rango es 4 ¡Cumple con la Condición!

%-Diseno mediante LQR -----------------------------------------------------
auto_val=eig(Ma);
% Coeficientes del poliniomio característico de A (Lazo Abierto) ai 
ai_A=conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 -auto_val(4)]); 
 
% Matriz W (3x3)
Wa=[ai_A(4) ai_A(3) ai_A(2)  1 ;
    ai_A(3) ai_A(2)    1     0 ;
    ai_A(1)    1       0     0 ;
    1          0       0     0]; 
  
% Defino la Matriz de trasformación para obtener una forma canónica 
% controlable. 
Ta=Ma*Wa; 

% Verificación que T posee una inversa. Forma canónica controlable. 
A_c=inv(Ta)*Aa*Ta;   % Matriz A controlable 
                     % Verificación de T. Se puede calcular.
%CALCULO DEL CONTROLADOR K
%para el calculo del mismo se utiliza el metodo LQR para lo cual definimos
Q=diag([1 1 1/1e9 1e4]);
R=9; %
[Ka,Pa,Ea]=lqr(Aa,Ba,Q,R); % Controlador ampliado
K_i= -Ka(4);       % Ka=[K -Ki ]
K=Ka(1:3);

% Defino el Torque dado y valor inicial
T_L=1.15e-3; 

% Defino a todas la variables de estudio con la misma cantidad de valores:
t=linspace(0,t_S,paso);
ia=linspace(0,t_S,paso);
theta=linspace(0,t_S,paso);
w=linspace(0,t_S,paso);
u=linspace(0,t_S,paso);
psi=linspace(0,t_S,paso);
y_sal=linspace(0,t_S,paso);
% Referencia
ref=(pi/2)*square(2*pi*t/20);
% Torque
TL=(T_L/2)+(T_L/2)*square(2*pi*t/20);

% Condiciones Iniciales
u(1)=0; ang_ref=pi/2;psi(1)=0;psi_p=0;
ia(1)=0;theta(1)=0;w(1)=0;ref(1)=0;wp=0;

% Simulación.-------------------------------------------------------------- 
ii=1;
while(ii<paso)  
    % Matriz de estados del sistema. 
    Estados=[ia(ii); w(ii); theta(ii)];
    % Acción de control.   
    psi_p=ref(ii)-C* Estados;
    psi(ii+1)=psi(ii)+psi_p*h;
    u(ii)=-K*Estados+K_i*psi(ii+1); 
    % Sistema no lineal 
    iap=(-Ra*Estados(1)-Km*Estados(2)+u(ii))/Laa;
    wp=(Ki*Estados(1)-Bm*Estados(2)-TL(ii))/J; 
    thetap=Estados(2);
    % Parte sin observador
    xp=[iap;wp;thetap];
    xp_next=Estados(1:3)+h*xp;
    ia(ii+1)   =xp_next(1);
    w(ii+1)    =xp_next(2);
    theta(ii+1)=xp_next(3); 
    y_sal(ii)=C*Estados; ii=ii+1;  
end 

figure(1);hold on; color='r'; 
subplot(3,1,1);plot(t,ref,'k');hold on;
plot(t,theta,color);grid on;hold on; 
title('θ_t');xlabel('segundos');hold on; 
hold on; color='b';
subplot(3,1,2);plot(t,ia,color);grid on; 
title('Corriente i_a');xlabel('segundos');hold on; 
color='g';hold on;
subplot(3,1,3);plot(t,u,color);grid on; 
title('Acción de control u(t)');xlabel('segundos');hold on;
