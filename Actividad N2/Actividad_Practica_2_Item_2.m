clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº2 Diseño de controladores en variables 
% de estado en tiempo continuo
% ===================================================================

Laa=5e-3; J=0.004; Ra=0.2; 
Bm=0.005; Ki=6.5e-5; Km=0.055;

% Defino las Matrices del sistema
A=[-Ra/Laa -Km/Laa 0; Ki/J -Bm/J 0; 0 1 0];
B=[1/Laa; 0; 0];
C=[0 0 1];
D=0;
% Calculos los autovalores (son tres)
autovalores_A=eig(A);
sigma1=autovalores_A(1);   % sigma1=0
sigma2=autovalores_A(2);   % sigma2=-1.2546
sigma3=autovalores_A(3);   % sigma3=-39.9954

% Tomo el autovalor con la dinámica más rápida y el más lento

% t_I=log(0.95)/sigma3;    % tr=0.00128
h=6e-04;                   % defino un valor dos veces más chico
% t_S=log(0.05)/sigma2;    % t_s=2.3878
t_S=8;                     % defino un valor dos veces más grande
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
Q=diag([0.001 1 1 1e14]);
R=0.00001; %
[Ka,Pa,polos_deseados]=lqr(Aa,Ba,Q,R); % Controlador ampliado
K_i= -Ka(4);       % Ka=[K -Ki ]
K=Ka(1:3);
% Polos Deseados:
mui=polos_deseados;
disp('Controlador ampliado en ') 
eig(Aa-Ba*Ka) 
% Calculo del Observador---------------------------------------------------
A_o=A';
B_o=C';
C_o=B';
%CALCULO DEL CONTROLADOR Ko
%para el calculo del mismo se utiliza el metodo LQR para lo cual definimos
Qo=diag([1 1 1]);
Ro=0.001; %
[K_o,Po,polos_deseados_o]=lqr(A_o,B_o,Qo,Ro); % Controlador ampliado
mui_o=polos_deseados_o;
Ko=K_o';
disp('Observador en:')
eig(A-Ko*C)
% Defino el Torque dado y valor inicial
T_L=1.15e-3; 

% Defino a todas la variables de estudio con la misma cantidad de valores:
t=linspace(0,t_S,paso);
ia=linspace(0,t_S,paso);
theta=linspace(0,t_S,paso);
w=linspace(0,t_S,paso);
u=linspace(0,t_S,paso);

% Ganancia de prealimentación de la referencia. 
Gj=-inv(C*inv(A-B*K)*B);

% Referencia
ref=(pi/2)*square(2*pi*t/4);
% Torque
TL=(T_L/2)+(T_L/2)*square(2*pi*t/4);

% Condiciones Iniciales
u(1)=0; ang_ref=pi/2;psi=0;
ia(1)=0;theta(1)=0;w(1)=0;ref(1)=0;wp=0;
x_hat=[0;0;0]; %Inicializo el Observador.

% Simulación.-------------------------------------------------------------- 
jj=0; 
while jj<2
    ii=1;
while(ii<paso)  
    % Matriz de estados del sistema. 
    Estados=[ia(ii); w(ii); theta(ii)]; 
    % Acción de control. 
    if jj==0
     u(ii)=-K*Estados+Gj*ref(ii); color='r';% Sin Observador
    else
     u(ii)=-K*x_hat+Gj*ref(ii); color='b';% Con Observador
    end
    % Sistema no lineal 
    wpp=(-wp*(Ra*J+Laa*Bm)-w(ii)*(Ra*Bm+Ki*Km)+u(ii)*Ki)/(J*Laa); 
    iap=(-Ra*ia(ii)-Km*w(ii)+u(ii))/Laa; 
    wp=wp+h*wpp-TL(ii)/J; 
    ia(ii+1)=ia(ii)+h*iap; 
    w(ii+1)=w(ii)+h*wp; 
    theta(ii+1)=theta(ii)+h*w(ii); 
   
    % Parte Observable
    y=C*Estados;
    y_o(ii)=C*x_hat; 
    x_hat_p=A*x_hat+B*u(ii)+Ko*(y-y_o(ii)); 
    x_hat=x_hat+x_hat_p*h; 
    
    ii=ii+1; 
end 

y_o(ii)=0;
figure(1);hold on; 
subplot(3,1,1);plot(t,ia,color);grid on;hold on; 
title('Corriente i_a');
if color=='r'
legend('Sin Observador');
else
legend('Con Observador');
end
subplot(3,1,2);plot(t,ref,'k');hold on;
plot(t,theta,color);grid on;hold on;
title('Ángulo del eje \theta');hold on; 
subplot(3,1,3);plot(t,u,color);grid on;hold on;
title('Acción de control u_t');xlabel('Tiempo en Seg.');
jj=jj+1;
end
