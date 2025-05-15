clear all;

datos=xlsread("D:\Usuario\Documents\4°to AÑO\Teoria de Control ll\trabajos con Matlab 2025/Curvas_Medidas_Motor_2024_v.xls"); %Cargamos los datos proporcionados

vector=transpose(datos);
filas=length(vector); %Obtenemos la cantidad de elementos

% Inicializamos los vectores, con la cantidad de filas que posee el archivo 
% transpuesto, y solo 
w = zeros(1,filas); %entrada
I_armadura = zeros(1,filas);
tiempo=zeros(1,filas);
tension=zeros(1,filas);
Torque=zeros(1,filas);

for i=1:1:5
    info{i} = vector(i, :);
end

tiempo=info{1};
w=info{2};
I_armadura=info{3};
tension=info{4};
torque=info{5};

StepAmplitude=12;
%%
%INICIO DE CALCULO DE CHEN PARA W/Va%
 

t_inic=0.0351;  
t_retardo=0.03503; %Tiempo donde inicia la dinamica de la curva de la velocidad angular con respecto a Va
t_diferencia=t_inic-t_retardo;

[val, lugar] =min(abs(t_diferencia+t_retardo-tiempo)); %Con esta operacion buscamos dentro del vector tiempo, le que se nos                                    
t_t1=tiempo(lugar);                                   %proporciona como dato, el tiempo mas cercano a la op t_dif+t_ret.
y_t1=w(lugar);                                    %Una vez obtenido se nos devuelve el indice de la posicion para igresar con el resto de datos

ii=1;
[val, lugar] =min(abs(2*t_diferencia+t_retardo-tiempo));
t_2t1=tiempo(lugar);
y_2t1=w(lugar);

[val, lugar] =min(abs(3*t_diferencia+ t_retardo-tiempo));
t_3t1=tiempo(lugar);
y_3t1=w(lugar);

K=198.2488/StepAmplitude; %La funcion al final de su trayectoria tiene una salida de 198.2488 radd/s, la cual es  a la que gira en vacio
k1=(1/StepAmplitude)*(y_t1/K)-1;%Afecto el valor del Escalon
k2=(1/StepAmplitude)*(y_2t1/K)-1;
k3=(1/StepAmplitude)*(y_3t1/K)-1;

be=4*k1^3*k3-3*k1^2*k2^2-4*k2^3+k3^2+6*k1*k2*k3;

alfa1=(k1*k2+k3-sqrt(be))/(2*(k1^2+k2));
alfa2=(k1*k2+k3+sqrt(be))/(2*(k1^2+k2));

 beta=(k1+alfa2)/(alfa1-alfa2);

%beta=(2*k1^3+3*k1*k2+k3-sqrt(be))/(sqrt(be)); %Beta para dos polos distintos y un cero



T1_ang=-t_diferencia/log(alfa1);
T2_ang=-t_diferencia/log(alfa2);
T3_ang=beta*(T1_ang-T2_ang)+T1_ang;

T1(ii)=T1_ang;
T2(ii)=T2_ang;
T3(ii)=T3_ang;

T3_ang=real(sum(T3/length(T3)));
T2_ang=real(sum(T2/length(T2)));
T1_ang=real(sum(T1/length(T1)));

FdT_chen_w_v=tf(K*[T3_ang 1],conv([T1_ang 1],[T2_ang 1]))%Funcion que nos devuelve la funcion de Transferencia
%Esta representa la FdT de W(s)/Tension

%FIN DE CALCULO DE CHEN PARA W/Va

%% 
%Hago una equivalencia numeria para luego grafica estos puntos, los puntos
%de Chen en la grafica de la W/va
t1_va=t_t1;
yt1_va=y_t1;

t2_va=t_2t1;
y2t1_va=y_2t1;

t3_va=t_3t1;
y3t1_va=y_3t1;

%Obtenemos el tiempo de simulacion
h_va=log(0.95)/(2*max(roots(conv([T1_ang 1],[T2_ang 1]))));

[w_va_chen, t__w_chen]=lsim(FdT_chen_w_v,tension,tiempo,[0 0]);

%%
%ETAPA DE PRE-CALCULO PARA W/TL

%La gaancia de K sera de 198-164=34 debido a que consideramos las condiciones
%inicales del motor que esta en funcionamiento cuando se le aplica el
%torque, por ende, se toma la diferencia provocada por la ganancia del
%torque mismo TL

w_con_torque=torque(18517:33502);
StepAmplitudeTL= mean(w_con_torque);

%Tomamos un valor promedio de la entrada, que en este caso queremos evaluar
%la velocidad con respecto a la entrada TL. como dicha entrada posee ruido,
%tomamos un promedio, que lo llamaremos StepAmplitudeTL

%%
%INICIO DE CALCULO DE CHEN PARA W/TL%
 

t_inic=0.18502;  %Tomando este tiempo es que se logro la mejor aproximacion a la curva original
t_retardo=0.18501; %Tiempo donde inicia la dinamica debido al efecto de la carga TL
t_diferencia=t_inic-t_retardo;

[val, lugar] =min(abs(t_diferencia+t_retardo-tiempo)); %Con esta operacion buscamos dentro del vector tiempo, le que se nos                                    
t_t1=tiempo(lugar);                                    %proporciona como dato, el tiempo mas cercano a la op t_dif+t_ret.
y_t1=w(lugar);                                         %Una vez obtenido se nos devuelve el indice de la posicion para igresar con el resto de datos

ii=1;
[val, lugar] =min(abs(2*t_diferencia+t_retardo-tiempo));
t_2t1=tiempo(lugar);
y_2t1=w(lugar);

[val, lugar] =min(abs(3*t_diferencia+ t_retardo-tiempo));
t_3t1=tiempo(lugar);
y_3t1=w(lugar);


%

K=34.6806/StepAmplitudeTL; %La funcion al final de su trayectoria tiene una salida de 12V, ya qe es la tension a la que se carga el cap
k1=(1/StepAmplitudeTL)*(y_t1/K)-1;%Afecto el valor del Escalon
k2=(1/StepAmplitudeTL)*(y_2t1/K)-1;
k3=(1/StepAmplitudeTL)*(y_3t1/K)-1;

be=4*k1^3*k3-3*k1^2*k2^2-4*k2^3+k3^2+6*k1*k2*k3;

alfa1=(k1*k2+k3-sqrt(be))/(2*(k1^2+k2));
alfa2=(k1*k2+k3+sqrt(be))/(2*(k1^2+k2));

 beta=(k1+alfa2)/(alfa1-alfa2);

T1_ang=-t_diferencia/log(alfa1);
T2_ang=-t_diferencia/log(alfa2);
T3_ang=beta*(T1_ang-T2_ang)+T1_ang;

T1(ii)=T1_ang;
T2(ii)=T2_ang;
T3(ii)=T3_ang;

T3_ang=0.05*real(sum(T3/length(T3))); %Se considero que el valor de 0.05 permite una comparacion optima del ruido provocado por la carga
T2_ang=real(sum(T2/length(T2)));      % EN LA GRAFICA SIMULADA en comparacion con la curva real de la carga TL
T1_ang=real(sum(T1/length(T1)));

FdT_chen_w_TL=tf(K*[T3_ang 1],conv([T1_ang 1],[T2_ang 1]))%Funcion que nos devuelve la funcion de Transferencia
%Esta representa la FdT de W(s)/Tension

%FIN DE CALCULO DE CHEN PARA W/TL

%%

[w_tl_chen, t_chen, entrada]=lsim(FdT_chen_w_TL,torque,tiempo,[0 0]); %con uste funcion, yo lo que hago es simular la funcion de transferencia en base a parametros de entrada, para luego guardarlo en vectores para su posterior ploteoo


%%
%Seccion de calculo

simulada=w_va_chen-w_tl_chen;
%%



%subplot(4,1,1)
plot(tiempo,w,'r'),hold on, plot(t__w_chen,simulada,'b'),title('Veloidad'); %,plot(t_chen,w_tl_chen,'g'), plot(t__w_chen,w_va_chen, 'b')
legend('Curva real','Curva aproximada con Chen','Puntos de Va','Puntos de TL');

%Ploteamos los puntos marcados
plot(t1_va,yt1_va,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r',  'HandleVisibility','off')
plot(t2_va,y2t1_va,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r',  'HandleVisibility','off')
plot(t3_va, y3t1_va,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');

 plot(t_t1,y_t1,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g',  'HandleVisibility','off')
 plot(t_2t1,y_2t1,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g',  'HandleVisibility','off')
 plot(t_3t1,y_3t1,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g')


% subplot(4,1,2)
% plot(tiempo,I_armadura),title('Corriente');
% 
% subplot(4,1,3)
% plot(tiempo,tension),title('Tension');
% 
% subplot(4,1,4)
% plot(tiempo,torque),title('Torque');


