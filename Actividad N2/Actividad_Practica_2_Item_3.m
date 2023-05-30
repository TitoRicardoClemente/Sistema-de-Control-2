clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº2 Diseño de controladores en variables 
% de estado en tiempo continuo
% ===============================================================
% CASO 2: Sistema no lineal de cuatro variables de estado 
% -------------------------------------------------------
% Valor de las Parámetros:
m=0.1;     % masa del péndulo
Fricc=0.1; % Fricción del rodado con la superficie
long=1.6;  % largo del pédulo
g=9.8;     % constante de la gravedad 
M=1.5;     % mase del carro

% Defino las Matrices del sistema
A=[0 1 0 0; 0 -Fricc/M -(m*g)/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/long*M 0];
B=[  0; 1/M; 0; -1/(long*M)];
C=[1 0 1 0]; % Salida en posición y ángulo
%-Calculo de At------------------------------------------------------------

% Autovalores de A:
autovalores_A=eig(A);

sigma1=autovalores_A(1);   % sigma1=0
sigma2=autovalores_A(2);   % sigma2=-0.0648
sigma3=autovalores_A(3);   % sigma3=-3.8350

% Tomo el autovalor con la dinámica más rápida y el más lento

t_I=log(0.95)/sigma3;    % tr=0.00134
h=5e-3;                 % defino un valor tres veces más chico At=0.005
%t_S=log(0.05)/sigma2;   % t_s=2.3878
t_S=20;                  % defino un valor dos veces más grande
tiempo=round(t_S/h);     % 4000

% Prueba de Condición de Controlabilidad:

Mc=[B, (A*B), (A^2)*B (A^3)*B];
rangoM= rank(Mc);        % el rango es 4 ¡Cumple con la Condición!

%-Diseno LQR --------------------------------------------------------------

% Coeficientes del poliniomio característico de A (Lazo Abierto) ai 
ai_A=conv(conv(conv([1 -autovalores_A(1)],[1 -autovalores_A(2)]),[1 -autovalores_A(3)]),[1 -autovalores_A(4)]); 
 
% Matriz W (3x3)
W=[ai_A(4) ai_A(3) ai_A(2) 1 ;
   ai_A(3) ai_A(2) 1       0 ;
   ai_A(1) 1       0       0 ;
   1       0       0       0];
  
% Defino la Matriz de trasformación para obtener una forma canónica 
% controlable. 
T=Mc*W; 

% Verificación que T posee una inversa. Forma canónica controlable. 
A_c=inv(T)*A*T;   % Matriz A controlable 
  
% Ubicación de los polos de lazo cerrado en mui: 
% Defino la matriz Q como la identidad:
Q=diag([1 1 1 1]);
% Defino la Matriz R como la 
R=0.9;
% Aleternativa a la función LQR
% P=care(A,B,Q,R);
% K=inv(R)*B'*P;
[K,P,E]=lqr(A,B,Q,R);

% Polos del sistema
mui1=round(E(1)); % -1
mui2=round(E(2)); % -1
mui3=round(E(3)); % -4
mui4=round(E(4)); % -4

% Ganancia de prealimentación de la referencia. 
Gj=-inv(C*inv(A-B*K)*B); % Gj=-1.0541

% Verifica la ubicación de los polos:
Polos_controlados=eig(A-B*K);

% Variables:
t=linspace(0,t_S,tiempo);
p=linspace(0,t_S,tiempo); 
p_p=linspace(0,t_S,tiempo); 
p_pp=linspace(0,t_S,tiempo);
tita_pp=linspace(0,t_S,tiempo);
u=linspace(0,t_S,tiempo);
%ref=linspace(0,t_S,tiempo);
omega=linspace(0,t_S,tiempo);
alfa=linspace(0,t_S,tiempo);

%Condiciones Iniciales:
for phi=[0.2 0.5 0.7 1 1.3]
p_p(1)=0; p_pp(1)=0; alfa(1)=phi;
tita_pp(1)=0; ref=-10; omega(1)=0;

ii=1;
while (ii<tiempo)
    % Definición de Variables de estados
    estado=[p(ii); p_p(ii); alfa(ii); omega(ii) ];
    % Acción de Control 
    u(ii)=-K*estado+Gj*ref;
    % Sistema no lineal
    p_pp=(1/(M+m))*(u(ii)-m*long*tita_pp*cos(alfa(ii))+m*long*omega(ii)^2*sin(alfa(ii))-Fricc*p_p(ii)); 
    tita_pp(ii+1)=(1/long)*(g*sin(alfa(ii))-p_pp(ii)*cos(alfa(ii))); 
    p_p(ii+1)=p_p(ii)+h*p_pp(ii); 
    p(ii+1)=p(ii)+h*p_p(ii); 
    omega(ii+1)=omega(ii)+h*tita_pp(ii); 
    alfa(ii+1)=alfa(ii)+h*omega(ii);
    
    ii=ii+1;
end
figure(1);hold on;
subplot(2,1,1);plot(t,omega);grid on; title('Velocidad Angular dΦ/dt');xlabel('segundos');hold on;hold on; 
subplot(2,1,2);plot(t,alfa);grid on;title('Ángulo Φ');xlabel('segundos');hold on;
figure(2);hold on;
subplot(2,1,1); plot(t,p);grid on;title('Posición carro δ');xlabel('segundos');hold on;hold on; 
subplot(2,1,2);plot(t,p_p);grid on;title('Velocidad carro dδ/dt');xlabel('segundos');hold on;hold on; 
figure(3)
plot(t,u);grid on;title('Acción de control');xlabel('segundos');hold on; 
end