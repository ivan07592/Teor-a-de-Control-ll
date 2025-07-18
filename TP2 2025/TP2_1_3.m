clear all;

% Universidad Nacional de Catamarca. Facultad de Tecnologia y Ciencias
% Aplicadas
% Alumno: Monroy Acosta Jose Ivan
% M.U:00881 

% Item [1] Implementar un sistema en variables de estado que controle el angulo del motor, para
% consignas de pi/2 y -pi/2 cambiando cada 5 segundos y que el TL es el descripto en la planilla de datos
% comparando el desempeno con el obtenido con el PID digital del TP Nro 1. Hallar el valor de
% integracion Euler adecuado.
% Objetivo: acelerar la dinamica del controlador verificando el resultado con las curvas del archivo xlsx
% adjunto.

% 1- Evitando que la tension supere los 24 Volts en valor absoluto, especificar el tiempo de muestreo
% necesario para que el controlador cumpla el objetivo
% 2- Asumiendo que no puede medirse directamente la corriente, pero si la velocidad y el angulo,
% proponer un controlador que logre el objetivo.
% 3- Determinar el efecto de la nolinealidad en la acción de control, descripta en la Fig. 2, y verificar
% cuál es el máximo valor admisible de ésa no linealidad.

% item [2] Implementar el mismo sistema controlador del item anterior, empleando los valores del
% motor siguiente,
% Laa=500 10-6; Jm=2.5 10-9; Ra=20; Bm=0; Ki=10 10-3; Kb=60.53 10-3;
% Generar graficas de evolución de la velocidad y angulo comparando los desempeños cualitativamente
% para todos los casos.

%%
%Definicion de paramteros:
TL_Max =0.0011;
X=-[0; 0;0];ii=0;
t_etapa =   1.0000e-05;
titaRef=pi/2;
tF=.30; %10 segundos
 
%Constantes del PID

Kp=20;Ki=250;Kd=0.0001;color_='b';
Ts=t_etapa;
A1=((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1=(-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1=Kd/Ts;
N=round(tF/t_etapa);
e=zeros(N,1);u=0;k=2;TL_ap=0;
um=20;

for jj=1:N
    ii=ii+1;k=k+1;

    %Aplicamos las condiciones del torque en funcion de lo requerido por el
    % ejercicio
    if jj*t_etapa>.07 %Primeros 5seg con Torque nulo
        TL_ap=TL_Max;
    end
    if jj*t_etapa<=.15 %Primeros 5seg con Torque nulo
        titaRef=pi/2;        
    end
    if jj*t_etapa>.15
        titaRef=-pi/2;
        TL_ap=0;
    end
    if jj*t_etapa>.23 && jj*t_etapa<.28
        TL_ap=TL_Max;
    end 

%     el PID, desde cálculo
    % X=modmotor_identificado(t_etapa, X, [u,TL_ap]); %Con ésto se prueba (simil planta)
    X=Funcion_motor(t_etapa, X, [u,TL_ap]);
    e(k)=titaRef-X(3); %ERROR tita
    u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
    ui=u;
    if abs(ui)<um %um es zona muerta
        u=0;
    else
        u=ui-um*sign(u);
    end  
    x1(ii)=X(1);% 
    x2(ii)=X(2);% 
    x3(ii)=X(3);%X=[ia;wr;titar];

   acc(ii)=u;%sin zona muerta
    acci(ii)=ui; %con zona muerta
    TL_D1(ii)=TL_ap;
    titaRef_(ii)=titaRef;
end
th=0:t_etapa:tF;
t=th(1:ii);
figure(1);
sgtitle('Controlador PID');
subplot(4,1,1);hold on;
plot(t,titaRef_,'--' ,t,x3,color_);title('Salida y, \theta_t');legend('Ref','\theta');legend('boxoff');grid on;
subplot(4,1,2);hold on;grid on;
plot(t,x1,color_);title('Corriente i_t');
xlabel('Tiempo [Seg.]');
subplot(4,1,3);hold on;grid on;
plot(t,TL_D1,color_);title('Torque T_L');
xlabel('Tiempo [Seg.]');
subplot(4,1,4);hold on;grid on;
plot(t,acc,'k',t,acci,color_);title('Entrada u_t, v_a');
xlabel('Tiempo [Seg.]');grid on;
 legend('u(t)S_zm','ui(t)');

%%
% break
%%%%%%%%%
%Control en el Estacio de estados
%%%%%%%%%%
% Según el tiempo de muestreo que es 1e-3, lo máximo que puede ir a la
% izquierda algún polo es 3/1e-3
X = -[0; 0; 0]; ii = 0; k = 2; psi = 0;

disp('Valor a la izquierda máximo es:')
1/(3*t_etapa)
% 3.3333e+04
% # Constantes Identificadas
Ra= 19.49908887350271; Laa=0.00046795934775014573; Ki=0.009885263081304892;
Jm= 1.5081203235402389e-09; Bm= 0; Km = 0.06053000001429517;


%Determinando las matrices A, B, B_T, y C
Mat_A=[-Ra/Laa -Km/Laa 0;
    Ki/Jm -Bm/Jm 0;
    0 1 0 ];
Mat_B=[1/Laa;0;0]; Mat_B_T=[0;-1/Jm;0];
Mat_C=[0 0 1;0 1 0]; %Sale tita, y omega.


%%
% break
% En estados, con matrices ampliadas
%%%%%%%%% Cálculo del Controlador LQR %%%%%%%%%%%%%%%%%%%

%Como primer paso para construir nuestro controlador LQR es armar nuestras
%matrices ampliadas:
% Sistema ampliado para referencia no nula

Aa=[Mat_A,[0;0;0];-Mat_C(1,:),0]; %ojo, el valor deseado es tita
Ba=[Mat_B;0];   
Mat_M=[Ba Aa*Ba Aa^2*Ba Aa^3*Ba]; %Matriz Controlabilidad
disp('Debe ser 4:')
rank(Mat_M) %Debe ser 4

%DEFINIMOS EL CONTROLADOR LQR%
Ka=lqr(Aa,Ba,diag([2e-2 1e-5 1e-5  2.6e5]),1e-2);

eig(Aa-Ba*Ka) % Polos del LQR


%% Cálculo del Observador
%Sistema Dual
Ad=Mat_A';Cd=Mat_B';
hh=[0.5;0.5];% matriz que se usa para la asignacion de polos del controlador
Bd=Mat_C'*hh;

% Sistema ampliado para referencia no nula
 
Mat_Md=[Bd Ad*Bd Ad^2*Bd  ]; %Matriz Controlabilidad
disp('Debe ser 3:')
rank(Mat_Md) %Debe ser 3, y mide sólo tita
auto_val=eig(Ad);
c_ai=poly(auto_val);
Mat_Wd=[ c_ai(3)  c_ai(2) 1  
       c_ai(2)  1     0 ;
       1  0     0 ];
Mat_Td=Mat_Md*Mat_Wd;

Ko=(lqr(Mat_A',Mat_C',diag([2e-2 1e0 1e3]),diag([1e-5, 1e-5])))';
eig(Mat_A-Ko*Mat_C) %Polos del observador con el controlador LQR

% break

%%
% Simulación del comportamiento dinámico
X=[0;0;0];
psi=0; 
x_hat=[0;0;0];
N=round(tF/t_etapa);
e=zeros(N,1);u=0;k=2;TL_ap=0;ii=0;k=2;
um=20;
ia_hat=zeros(1,N);

for jj=1:N
    ii=ii+1;k=k+1;

    %Aplicamos las condiciones del torque en funcion de lo requerido por el
    % ejercicio
    if jj*t_etapa>.07 %Primeros 5seg con Torque nulo
        TL_ap=TL_Max;
    end
    if jj*t_etapa<=.15 %Primeros 5seg con Torque nulo
        titaRef=pi/2;        
    end
    if jj*t_etapa>.15
        titaRef=-pi/2;
        TL_ap=0;
    end
    if jj*t_etapa>.23 && jj*t_etapa<.28
        TL_ap=TL_Max;
    end 
    Y=Mat_C*X;
    
    %el PID, desde cálculo
    % X=modmotor_identificado(t_etapa, X, [u,TL_ap]); %Con ésto se prueba (simil planta)
    X=Funcion_motor(t_etapa, X, [u,TL_ap]);

    e(k)=titaRef-Y(1); %ERROR tita
    u=-Ka *[x_hat;psi]; %Con observación de estados
    
    ui=u;
    if abs(ui)<um %um es zona muerta
        u=0;
    else
        u=ui-um*sign(u);
    end  
    x_hat_p = Mat_A*x_hat + Mat_B*u + Mat_B_T*TL_ap + Ko*(Y - Mat_C*x_hat);
    % x_hat_p=Mat_A*x_hat+Mat_B*u + Ko*(Y-Mat_C*x_hat);
    x_hat=x_hat+x_hat_p*t_etapa;

    Psi_punto=e(k);
    psi=Psi_punto*t_etapa +psi;

    x1(ii)=X(1);% 
    x2(ii)=X(2);% 
    x3(ii)=X(3);%X=[ia;wr;titar];
    ia_hat(ii)=x_hat(1);

    acc(ii)=u;%sin zona muerta
    acci(ii)=ui; %con zona muerta
    TL_D1(ii)=TL_ap;
    titaRef_(ii)=titaRef;

end

th=0:t_etapa:tF;
figure(2);
subplot(4,1,1);hold on;
plot(t,titaRef_,'--' ,t,x3,color_);title('Salida y, \theta_t');legend('Ref','\theta');legend('boxoff');grid on;
subplot(4,1,2);hold on;grid on;
plot(t,x1,color_);title('Corriente i_t');
plot(t,ia_hat,'r');
xlabel('Tiempo [Seg.]');
legend('ia','ia_hat');
subplot(4,1,3);hold on;grid on;
plot(t,TL_D1,color_);title('Torque T_L');
xlabel('Tiempo [Seg.]');
subplot(4,1,4);hold on;grid on;
plot(t,acc,'k',t,acci,color_);title('Entrada u_t, v_a');
xlabel('Tiempo [Seg.]');grid on;
 legend('u(t)S_zm','ui(t)');
sgtitle('Controlador LQR');