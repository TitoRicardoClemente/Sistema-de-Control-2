clc; clear all; close all; 
  
% Valor de las Parámetros:
m=0.1;     % masa del péndulo
Fricc=0.1; % Fricción del rodado con la superficie
long=1.6;  % largo del pédulo
g=9.8;     % constante de la gravedad 
M=1.5;     % mase del carro

% Versión linealizada en el equilibrio inestable. Sontag Pp 104. 
A=[0 1 0 0; 0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/(long*M) 0]; 
B=[0; 1/M; 0; -1/(long*M)]; 
C=[1 0 1 0]; % La salida monovariable es la posición. 

% Autovalores de A:
autovalores_A=eig(A);
sigma1=autovalores_A(1);   % sigma1=0
sigma2=autovalores_A(2);   % sigma2=-0.0648
sigma3=autovalores_A(3);   % sigma3=-3.8350

% Tiempos de simulación. 
% Tomo el autovalor con la dinámica más rápida y el más lento

t_I=log(0.95)/sigma3;    % tr=0.00134
h=5e-3;                 % defino un valor tres veces más chico At=0.005
%t_S=log(0.05)/sigma2;   % t_s=2.3878
t_S=20;                  % defino un valor dos veces más grande
tiempo=round(t_S/h);     % 4000 
t=linspace(0,t_S,tiempo);
% Matriz Controlabilidad. 
MC=[B A*B A^2*B A^3*B]; 
rango_M=rank(M); 
  
% Cálculo de controlador por asignación de polos.-------------------------- 
  
% Coeficientes del poliniomio característico original. 
c_ai=poly(A); 
  
% Matriz W por definición. 
W=[c_ai(4) c_ai(3) c_ai(2) 1; c_ai(3) c_ai(2) 1 0; c_ai(2) 1 0 0; 1 0 0 0]; 
  
% Matriz de trasformación. 
T=MC*W; 
  
% Verificación de que T esté bien. Forma canónica controlable. 
A_controlable=inv(T)*A*T; 
  
% Ubicación de los polos de lazo cerrado en mui.
mui(1)=-1; mui(2)=-1; mui(3)=-4; mui(4)=-4; 
  
% Coeficientes del poliniomio característico para controlar. 
alfa_i=poly(mui); 
  
% Cálculo del Controlador K. 
K=fliplr(alfa_i(2:end)-c_ai(2:end))*inv(T); 
% K=[-3.9184,-9.896,-1.0115e+02,-39.6735]
% Ganancia de prealimentación para referencia no nula. 
Gj=-inv(C*inv(A-B*K)*B); 
% Gj=-3.9184
% Verifica la ubicación de los polos. 
polos_controlados=eig(A-B*K); 
  
% Diseño de observador, usando propiedad de dualidad.---------------------- 
Mat_A_O=A'; 
Mat_B_O=C'; 
  
% Matriz Controlabilidad. 
Mat_M_Dual=[Mat_B_O Mat_A_O*Mat_B_O Mat_A_O^2*Mat_B_O Mat_A_O^3*Mat_B_O]; 
  
% Analisis del rango para determinar si es observable. 
rango_M_Dual=rank(Mat_M_Dual); 
alfaO_i=alfa_i; 
  
% Ubicacion del Observador. 
mui_o=real(mui)*0.2; 
  
alfaO_i=poly(mui_o); 
Mat_T_O=Mat_M_Dual*W; 
Ko=(fliplr(alfaO_i(2:end)-c_ai(2:end))*inv(Mat_T_O))'; 
  
% Verifica la ubicación de los polos. 
eig(Mat_A_O'-Ko*C); 
x_hat=[0;0;0;0]; %Inicializo el Observador. 

d_pp=0; tita_pp=0; 
omega=linspace(0,0,tiempo); 
alfa=linspace(0,0,tiempo); 
d=linspace(0,0,tiempo); 
d_p=linspace(0,0,tiempo); 
u=linspace(0,0,tiempo); 
% Condiciones iniciales. 
alfa(1)=.1; 
omega(1)=0; d_p(1)=0; d(1)=0; 
u(1)=0; ii=1; 
referencia=-10; 

% Simulación. 
while(ii<(tiempo+1)) 
    % Variables del sistema no lineal. 
    estado=[d(ii); d_p(ii); alfa(ii); omega(ii)]; 
     
    % Acción de control. 
    %u(ii)=-K*estado+Gj*referencia; color='r';% Sin Observador. 
    u(ii)=-K*x_hat+Gj*referencia; color='b';% Con Observador. 
     
    % Sistema no lineal. 
    d_pp=(1/(M+m))*(u(ii)-m*long*tita_pp*cos(alfa(ii))+m*long*omega(ii)^2*sin(alfa(ii))-Fricc*d_p(ii)); 
    tita_pp=(1/long)*(g*sin(alfa(ii))-d_pp*cos(alfa(ii))); 
    d_p(ii+1)=d_p(ii)+h*d_pp; 
    d(ii+1)=d(ii)+h*d_p(ii); 
    omega(ii+1)=omega(ii)+h*tita_pp; 
    alfa(ii+1)=alfa(ii)+h*omega(ii); 
     
    % Observador----------------------------------------------------------- 
    y_sal_O(ii)=C*x_hat; 
    y_sal(ii)=C*estado; 
    x_hatp=A*x_hat+B*u(ii)+Ko*(y_sal(ii)-y_sal_O(ii)); 
    x_hat=x_hat+h*x_hatp; 
     
    ii=ii+1; 
end 
d_p(ii)=[]; d(ii)=[]; 
omega(ii)=[]; alfa(ii)=[];

figure(1);hold on;
subplot(2,1,1);plot(t,omega,color);grid on; title('Velocidad Angular dΦ/dt');xlabel('segundos');hold on;hold on; 
subplot(2,1,2);plot(t,alfa,color);grid on;title('Ángulo Φ');xlabel('segundos');hold on;hold on; 
figure(2);hold on;
subplot(2,1,1); plot(t,d,color);grid on;title('Posición carro δ');xlabel('segundos');hold on;hold on; 
subplot(2,1,2);plot(t,d_p,color);grid on;title('Velocidad carro dδ/dt');xlabel('segundos');hold on;hold on; 
figure(3)
plot(t,u,color);grid on;title('Acción de control');xlabel('segundos');hold on; 
