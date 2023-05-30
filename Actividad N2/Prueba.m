%clear all; close all; clc;
syms s t real
% Tito Ricardo Clemente
% Ingeniería Electronica
% Sistema de Control II - 2023
% 1. Actividad Práctica Nº2 Diseño de controladores en variables 
% de estado en tiempo continuo
% ===============================================================
% CASO 2: Sistema no lineal de cuatro variables de estado 
% -------------------------------------------------------
clc; clear all; close all; 
  
  
% Valor de las Parámetros:
m=0.1;     % masa del péndulo
Fricc=0.1; % Fricción del rodado con la superficie
long=1.6;  % largo del pédulo
g=9.8;     % constante de la gravedad 
M=1.5;     % mase del carro

% Tiempos de simulación. 
h=1e-4; t_simulacion=100; 
tiempo=(t_simulacion/h); 
t=linspace(0,t_simulacion,tiempo); 
  
% Declaracion de Variables de estudio. 
d_pp=0; fi_pp=0; 
omega=linspace(0,0,tiempo); 
fi=linspace(0,0,tiempo); 
d=linspace(0,0,tiempo); 
d_p=linspace(0,0,tiempo); 
u=linspace(0,0,tiempo); 
  
% Condiciones iniciales. 
fi(1)=pi; 
omega(1)=0; d_p(1)=0; d(1)=0; 
u(1)=0; ii=1; 
referencia=2; 
bandera=1; 
  
% Versión linealizada en el equilibrio estable. 
A=[0 1 0 0; 
       0 -Fricc/M -(m*g)/M 0; 
       0 0 0 1; 
       0 -Fricc/(long*M) -(g*(m+M))/(long*M) 0]; 
B=[0; 1/M; 0; 1/(long*M)]; 
C=[1 0 0 0]; % La salida monovariable es la posición. 
  
% Matriz Controlabilidad. 
Mat_M=[B A*B A^2*B A^3*B]; 
rango_M=rank(Mat_M); 
  
% Cálculo del controlador LQR---------------------------------------------- 
Q=diag([0.9 15 6e-3 9e-6]); 
R=0.01;
  
% Contrucción del Hamiltoniano 
H=[A -B*inv(R)*B'; -Q -A']; 
[V,D]=eig(H);MX1X2=[]; 
for(jj=1:8) 
    if real(D(jj,jj))<0 
        MX1X2=[MX1X2 V(:,jj)]; 
    end 
end 
MX1=MX1X2(1:4,:); MX2=MX1X2(5:8,:); 
P=real(MX2*inv(MX1)); 
K=inv(R)*B'*P; 

% Ganancia de prealimentación para referencia no nula. 
Gj=-inv(C*inv(A-B*K)*B); 

% Verifica la ubicación de los polos. 
eig(A-B*K) ;

% Fin Calculo de controlador LQR para sistema sin Observador---------------
% Cálculo del controlador LQR para sistema con observador------------------ 
Ao=A';
Bo=C';
Co=B';

Qo=diag([0.9 15 6e-3 9e-6]); 
Ro=0.01;
  
% Contrucción del Hamiltoniano 
Ho=[Ao -Bo*inv(Ro)*Bo'; -Qo -Ao']; 
[no,vao]=size(Ho);
[V,D]=eig(Ho);MX1X2=[]; 
for(jj=1:8) 
    if real(D(jj,jj))<0 
        MX1X2=[MX1X2 V(:,jj)]; 
    end 
end 
MX1=MX1X2(1:no/2,:); MX2=MX1X2(((no/2)+1):end,:); 
Po=real(MX2*inv(MX1)); 
Ko=inv(Ro)*Bo'*Po;

estado=[d(1); d_p(1); fi(1); omega(1)]; 
x_hat=[0;0;0;0];
% Simulación. 
while(ii<(tiempo+1)) 
    % Variables del sistema no lineal. 
    estado=[d(ii); d_p(ii); fi(ii); omega(ii)]; 
    if d(ii) > 2
        referencia=0; 
        m=1; 
    end 
    % Acción de control. 
    u(ii)=-K*estado+Gj*referencia; color='r';% Sin Observador. 
    %u(ii)=-K*x_hat+Gj*referencia; color='b';% Con Observador. 
     
    % Sistema no lineal. 
    d_pp=(1/(M+m))*(u(ii)-m*long*fi_pp*cos(fi(ii))+m*long*omega(ii)^2*sin(fi(ii))-Fricc*d_p(ii)); 
    fi_pp=(1/long)*(g*sin(fi(ii))-d_pp*cos(fi(ii))); 
    d_p(ii+1)=d_p(ii)+h*d_pp; 
    d(ii+1)=d(ii)+h*d_p(ii); 
    omega(ii+1)=omega(ii)+h*fi_pp; 
    fi(ii+1)=fi(ii)+h*omega(ii); 
    %ref(ii)=pi; 
    % Observador----------------------------------------------------------- 
    y_sal_O(ii)=C*x_hat; 
    y_sal(ii)=C*estado; 
    x_hatp=A*x_hat+B*u(ii)+Ko'*(y_sal(ii)-y_sal_O(ii)); 
    x_hat=x_hat+h*x_hatp; 
    ii=ii+1; 
end 
d_p(ii)=[]; d(ii)=[]; 
omega(ii)=[]; fi(ii)=[]; 
figure(1);hold on;
subplot(2,1,1);plot(t,omega);grid on; title('Velocidad Angular dΦ/dt');xlabel('segundos');hold on;hold on; 
subplot(2,1,2);plot(t,fi);grid on;title('Ángulo Φ');xlabel('segundos');hold on;
figure(2);hold on;
subplot(2,1,1);plot(t,d);hold on;grid on;title('Posición carro δ');xlabel('segundos');hold on;hold on; 
subplot(2,1,2);plot(t,d_p);grid on;title('Velocidad carro dδ/dt');xlabel('segundos');hold on;hold on; 
figure(3)
plot(t,u);grid on;title('Acción de control');xlabel('segundos');hold on; 