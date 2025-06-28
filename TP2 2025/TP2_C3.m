clc;clear all;

% Universidad Nacional de Catamarca. Facultad de Tecnologia y Ciencias
% Aplicadas
% Alumno: Monroy Acosta Jose Ivan
% M.U:00881 

% Calcular sistema controlador que haga evolucionar al péndulo en el equilibrio estable.
% Objetivo de control: partiendo de una condición inicial nula en el desplazamiento y el ángulo en , hacer que el carro se desplace a 10 metros evitando las oscilaciones de la masa m, considerando que es una grúa. Una vez que =10 modificar a m a un valor 10 veces mayor y volver al origen evitando oscilaciones.
% 1- Considerar que sólo puede medirse el desplazamiento  y el ángulo .
% 2- Especificar el rango posible para el tiempo de muestreo para implementar el sistema en un microcontrolador.
% 3- Determinar el efecto de la nolinealidad en la acción de control, descripta en la Fig. 1 (con dimensiones de fuerza en N), y verificar cuál es el máximo valor admisible de ésa no linealidad.
%%
% Datos del problema
m=.1;Fricc=0.1; long=0.6;g=9.8;M=.5;
TamanioFuente=12;


%Condiciones iniciales
alfa(1)=pi-.5; colorc='.g';colorl='g';
alfa(1)=pi; colorc='.r';colorl='r';

%%
%En base a las ecuaciones del pendulo proponemos realizar una linealizacion
%del sistema con el fin de poder operarlo

%Versión linealizada en el equilibrio inestable. Sontag Pp 104.
% estado=[p(i); p_p(i); alfa(i); omega(i)]
Mat_Ac=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/(long*M) 0];
Mat_Bc=[0; 1/M; 0; -1/(long*M)];

%% Mediante Taylor se linealiza en el punto de operación x_OP=[0;0;pi;0]
% Equilibrio estable
Mat_A=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(long*M) -g*(m+M)/(long*M) 0];
Mat_B=[0; 1/M; 0; 1/(long*M)];
xOP=[0;0;pi;0];
Mat_C=[1 0 0 0;0 0 1 0];%Mide delt y fi.

%% Cálculo del controlador. Se debe generar el sistema ampliado.
% Construcción del sistema ampliado
% Matrices del sistema con integrador
Mat_Aa = [Mat_A zeros(4,1); -Mat_C(1,:) 0];
Mat_Ba = [Mat_B; 0];
Qa = diag([1e-4 1e1 1e6 1e4 5e5]);  % Incluye el peso del integrador
Ra = 1e5;

% LQR para el sistema ampliado
Ka = lqr(Mat_Aa, Mat_Ba, Qa, Ra);

% Separar la ganancia de estado y del integrador
K  = Ka(1:4);
KI = -Ka(5);

% Verificación de polos del sistema en lazo cerrado
disp('Polos del controlador en:')
eig(Mat_Aa - Mat_Ba * Ka)


%% Cálculo del Observador
%Repito con el Sistema Dual
Mat_Adual=Mat_A';
Mat_Bdual=Mat_C';
Mat_Cdual=Mat_B'; 


Qdual = diag([1e1 1 1e2 1]);
Rdual = diag([1e-2, 1e-1]);

% Diseño del observador usando el sistema dual
Ko = lqr(Mat_Adual, Mat_Bdual, Qdual, Rdual)';

disp('Polos del observador en:')
eig(Mat_A-Ko*Mat_C)%  Polos del controlador con observacion de estados
% break

%% Tiempo de muestreo. Se fija luego de conocer la dinámica de LAZO CERRADO.
Ts = 1e-3;T=15;

KMAX=round(T/Ts);
t=0; x=[0;0;alfa(1);0];
p(1)=x(1); p_p(1)=x(2); alfa(1)=x(3); omega(1)=x(4);

tl=(0:KMAX-1)*Ts;
x=[0;0;alfa(1);0];
h=Ts;ref=10;tita_pp=0;x_hat=[0;0;0;0];

um=0 ; %Zona muerta máx es 2
delta_hat_pt=zeros(1,KMAX);
fi_hat_pt=zeros(1,KMAX);
for ki=1:KMAX
    estado=[p(ki); p_p(ki); alfa(ki); omega(ki)];
    Y_=Mat_C*estado;
    psi_p=ref-Mat_C(1,:)*estado;
    psi(ki+1)=psi(ki)+psi_p*h;

    u(ki)=-K*(x_hat-xOP)+KI*psi(ki+1);
    % u(ki)=-K*(estado-xOP)+KI*psi(ki+1);

    %Zona muerta
    ui=u(ki);    
    acci(ki)=ui;
    if abs(ui)<um %um es zona muerta
        u(ki)=0;
    else
        u(ki)=ui-um*sign(u(ki));
    end    
    %fin zona muerta

%%
% Parte de calculo%%

    p_pp=(1/(M+m))*(u(ki)-m*long*tita_pp*cos(alfa(ki))+m*long*omega(ki)^2*sin(alfa(ki))-Fricc*p_p(ki));
    tita_pp=(1/long)*(g*sin(alfa(ki))-p_pp*cos(alfa(ki)));
    p_p(ki+1)=p_p(ki)+h*p_pp;
    p(ki+1)=p(ki)+h*p_p(ki);
    omega(ki+1)=omega(ki)+h*tita_pp;
    alfa(ki+1)=alfa(ki)+h*omega(ki);
    %________OBSERVADOR__________%
    x_hatp=Mat_A*(x_hat-xOP)+Mat_B*u(ki)+Ko*(Y_-Mat_C*x_hat); %Restando el punto de operacion fuerzo a que el sistema se encuentre cerca del Punto de quilibrio
    x_hat=x_hat+h*x_hatp;

    % estado=[p(i); p_p(i); alfa(i); omega(i)]

    delta_hat_pt(ki)=x_hat(4);
    fi_hat_pt(ki)=x_hat(2);
end

%%

%% ETAPA DE RECORRIDO INVERSO

%Aqui definimos la masa de vuelta, como si se cargara el camion
m=1;

Mat_A=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(long*M) -g*(m+M)/(long*M) 0];
Mat_B=[0; 1/M; 0; 1/(long*M)];

Mat_Aa=[Mat_A zeros(4,1);-Mat_C(1,:) 0];
Mat_Ba=[Mat_B;0];


Qa = diag([1e1 1e1 1e6 1e4 1e4]);  % Incluye el peso del integrador
Ra = 1e5;

% LQR para el sistema ampliado
Ka = lqr(Mat_Aa, Mat_Ba, Qa, Ra);

% Separar la ganancia de estado y del integrador
K2=Ka(1:4); KI2=-Ka(5);

% Definición de pesos
% Qdual = diag([1e1 1 1e2 1]);
Qdual = diag([1e2 1e3 1e2 1e3]);  % Poner más énfasis en ṗ y ω

Rdual = diag([1e-2, 1e-1]);

% Diseño del observador usando el sistema dual
Ko2 = lqr(Mat_Adual, Mat_Bdual, Qdual, Rdual)';

disp('Polos del controlador en vuelta:'); eig(Mat_A-Mat_B*K2)
disp('Polos del observador en vuelta:'); eig(Mat_A-Ko2*Mat_C)

%% SIMULACIÓN VUELTA
ref=0;   %con esto volvemos a un delta igual a cero
KMAX2=round(T/Ts);
p2(1)=p(end); p_p2(1)=p_p(end); alfa2(1)=alfa(end); omega2(1)=omega(end);
x_hat2=[0;0;0;0]; psi2(1)=psi(end); tita_pp=0;

delta_hat2_pt=zeros(1,KMAX2);
fi_hat2_pt=zeros(1,KMAX2);

for ki=1:KMAX2
    estado=[p2(ki); p_p2(ki); alfa2(ki); omega2(ki)];
    Y_=Mat_C*estado;
    psi_p=ref-Mat_C(1,:)*estado;
    psi2(ki+1)=psi2(ki)+psi_p*h;

    u2(ki)=-K2*(estado-xOP)+KI2*psi2(ki+1);
    ui=u2(ki); acci2(ki)=ui;
    if abs(ui)<um
        u2(ki)=0;
    else
        u2(ki)=ui-um*sign(u2(ki));
    end    

    p_pp=(1/(M+m))*(u2(ki)-m*long*tita_pp*cos(alfa2(ki))+m*long*omega2(ki)^2*sin(alfa2(ki))-Fricc*p_p2(ki));
    tita_pp=(1/long)*(g*sin(alfa2(ki))-p_pp*cos(alfa2(ki)));
    p_p2(ki+1)=p_p2(ki)+h*p_pp;
    p2(ki+1)=p2(ki)+h*p_p2(ki);
    omega2(ki+1)=omega2(ki)+h*tita_pp;
    alfa2(ki+1)=alfa2(ki)+h*omega2(ki);

    x_hatp=Mat_A*(x_hat2-xOP)+Mat_B*u2(ki)+Ko2*(Y_-Mat_C*x_hat2);
    x_hat2=x_hat2+h*x_hatp;

    delta_hat2_pt(ki)=x_hat2(4);
    fi_hat2_pt(ki)=x_hat2(2);
end
%% GRAFICOS 
t=0:h:T;

t1=0:h:T; t2=0:h:T;
% t1=t1(end-1);
figure(1);
sgtitle('Sistema de Control de Pendulo');
subplot(3,2,1);plot(t1,alfa,'.r',t2,alfa2,'.b');grid on;title('\phi_t','FontSize',TamanioFuente);

subplot(3,2,2);plot(t1,omega,'.r',t2,omega2,'.b'), hold on;
plot(t1(1:end-1),delta_hat_pt,'g');%Simulacion del observador
plot(t2(1:end-1),delta_hat2_pt,'g')
grid on;title('$\dot{\phi_t}$','Interpreter','latex','FontSize',TamanioFuente);

subplot(3,2,3);plot(t1,p,'.r',t2,p2,'.b');grid on;title('\delta_t','FontSize',TamanioFuente);

subplot(3,2,4);plot(t1,p_p,'.r',t2,p_p2,'.b'), hold on;
plot(t1(1:end-1),fi_hat_pt,'g');
plot(t2(1:end-1),fi_hat2_pt,'g');
grid on;title('$\dot{\delta_t}$','Interpreter','latex','FontSize',TamanioFuente);

subplot(3,1,3);plot(t1(1:end-1),u,'.r',t2(1:end-1),u2,'.b');grid on;title('Acción de control','FontSize',TamanioFuente);xlabel('Tiempo en Seg.');
legend('Trayectoria de ida','Trayectoria de vuelta');