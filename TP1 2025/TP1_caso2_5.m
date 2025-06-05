clear all;

%Universidad Nacional de Catamarca. Facultad de Tecnologia y Ciencias
%Aplicadas
%Alumno: Monroy Acosta Jose Ivan
%M.U:00881

% �tem [5] A partir de las curvas de mediciones de las variables graficadas en la Fig. 1-3, se requiere 
% obtener el modelo del sistema considerando como entrada un escal�n de 12V, como salida a la 
% velocidad angular, y al torque de carga TL aplicado una perturbaci�n. En el archivo 
% Curvas_Medidas_Motor.xls est�n las mediciones, en la primer hoja los valores y en la segunda 
% los nombres. Se requiere obtener el modelo din�mico, para establecer las constantes del modelo 
% (1-5) (1-6).  

%%
datos=xlsread("D:\Usuario\Documents\4�to A�O\Teoria de Control ll\trabajos con Matlab 2025/Curvas_Medidas_Motor_2024_v.xls"); %Cargamos los datos proporcionados

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

tiempo = tiempo - tiempo(1); %Este arreglo se hace debido a que si el tiempo en el inicio no es cero, la funcion lsim nos da un error

StepAmplitude=12;
%%
%INICIO DE CALCULO DE CHEN PARA W/Va%
 

t_inic=0.0351;  
t_retardo=0.03503; %Tiempo donde inicia la dinamica de la curva de la velocidad angular con respecto a Va
t_diferencia=t_inic-t_retardo;

[~, lugar] =min(abs(t_diferencia+t_retardo-tiempo)); %Con esta operacion buscamos dentro del vector tiempo, le que se nos                                    
t_t1=tiempo(lugar);                                   %proporciona como dato, el tiempo mas cercano a la op t_dif+t_ret.
y_t1=w(lugar);                                    %Una vez obtenido se nos devuelve el indice de la posicion para igresar con el resto de datos

ii=1;
[~, lugar] =min(abs(2*t_diferencia+t_retardo-tiempo));
t_2t1=tiempo(lugar);
y_2t1=w(lugar);

[~, lugar] =min(abs(3*t_diferencia+ t_retardo-tiempo));
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

[~, lugar] =min(abs(t_diferencia+t_retardo-tiempo)); %Con esta operacion buscamos dentro del vector tiempo, le que se nos                                    
t_t1=tiempo(lugar);                                    %proporciona como dato, el tiempo mas cercano a la op t_dif+t_ret.
y_t1=w(lugar);                                         %Una vez obtenido se nos devuelve el indice de la posicion para igresar con el resto de datos

ii=1;
[~, lugar] =min(abs(2*t_diferencia+t_retardo-tiempo));
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
%Seccion de calculo de la respuesta completa de la Velocidid �ngular W para
%las entradas de torque y tension.

simulada=w_va_chen-w_tl_chen;

%%
%INICIO DE CALCULO DE CHEN MODIFICADO PARA I/Va%

%Aqui subyase un problema, y es que la curvade la corriente no presenta un
%valor en estado estacionario, por lo que no podemos aplicar de manera
%indicada el metodo de Chen, ademas de que su funcion de transferencia
%posee un polo en el origen. Esto nos lleva a introducir una variacion de
%dicho metodo (J. Pucheta et al, 2023)

  
t_retardo=0.03502; %Tiempo donde inicia la dinamica de la curva de la velocidad angular con respecto a Va


 %Recorte de los datos
Y_aux = I_armadura(31:16860);
t_aux = tiempo(31:16860);
u_aux = tension(31:16860);

y1i = max(Y_aux);
uMAX = max(tension);

% t1i, y1i
[~, lugar] = min(abs(Y_aux - y1i));
t1i = t_aux(lugar);
y1i = y1i / uMAX;

% t2i, y2i
[~, lugar] = min(abs((t1i - 0.03502)*2 + 0.03502 - tiempo));
t2i = tiempo(lugar);
y2i = I_armadura(lugar) / uMAX;

% t3i, y3i
[~, lugar] = min(abs((t1i - 0.03502)*3 + 0.03502 - tiempo));
t3i = tiempo(lugar);
y3i = I_armadura(lugar) / uMAX;

ii=1;

% C�lculo de alfa1 (seg�n f�rmula equivalente en MATLAB)
ret = 0.03502;
den = 4*y1i*y3i - 3*y2i^2;
alfa1_expr = (-4*y1i*y3i * sqrt(1/den) + 3*y2i^2 * sqrt(1/den) + y2i) / (2*y1i);
alfa1 = abs(alfa1_expr);

% alfa2
alfa2 = y2i / y1i - alfa1;

% Constantes de tiempo T1 y T2
T1 = -(t1i - ret) / log(alfa1);
T2 = -(t1i - ret) / log(alfa2);

% Par�metros del sistema
beta = y1i / (alfa1 - alfa2);
beta1 = y2i / (alfa1^2 - alfa2^2);
beta2 = y3i / (alfa1^3 - alfa2^3);  % Deber�a ser igual a beta y beta1

K_2_C = y1i * (T1 - T2) / (alfa1 - alfa2); % y1i ya normalizado

% Funci�n de transferencia
s = tf('s');
FdT_I_Va = K_2_C * (s) / ((T1*s + 1)*(T2*s + 1))

% Simulaci�n del sistema
[y_1, t_1, ~] = lsim(FdT_I_Va, tension, tiempo, [0; 0]);


%FIN DE CALCULO DE CHEN MODIFICADO PARA I/va

%%
%SECCION DE EXTRACCION DE PARAMETROS:

%compararemos los terminos de las funciones transferencia obtenidos con las
%teoricas para encontrar el valor de los parametros.Los terminos a obtener
%seran R, L, Km, J y Ki

% Su Funcion es (S/L)/(S^2 + SR/L + KmKi/JL)


%De la FdT de la CORRIENTE
num_IA=FdT_I_Va.Numerator{1};
den_IA=FdT_I_Va.Denominator{1};


NORMALIZADO_IA=den_IA(1);  %Factor de normalizacion

num_n_IA=num_IA/NORMALIZADO_IA;
den_n_IA=den_IA/NORMALIZADO_IA;

FdT_I_Va_n=tf(num_n_IA,den_n_IA)

Ian=num_n_IA(2);
Iad1=den_n_IA(1);
Iad2=den_n_IA(2);
Iad3=den_n_IA(3);


%Despejand de la FdT podemos llegar a que:

L=1/Ian
R=Iad2*L

%De la FdT de la VELOCIDAD ANGULAR CON RESPECTO A LA TENSION W/Va:
% Su Funcion es (Ki/J)/(S^2 + SR/L + KmKi/JL)

num_WVa=FdT_chen_w_v.Numerator{1};
den_WVa=FdT_chen_w_v.Denominator{1};

NORMALIZADO_WVa=den_WVa(1); %Factor de normalizacion

% ****Normalizamos numerador y denominador****
num_n_WVa=num_WVa/NORMALIZADO_WVa;
den_n_WVa=den_WVa/NORMALIZADO_WVa;

FdT_WVa_n=tf(num_n_WVa,den_n_WVa)

Wva_n2=num_n_WVa(3);
Wvad1=den_n_WVa(1);
Wvad2=den_n_WVa(2);
Wvad3=den_n_WVa(3);

Km=(Wvad3/Wva_n2)*L

%De la FdT de la VELOCIDAD ANGULAR CON RESPECTO AL TORQUE W/TL:
% Su Funcion es (S/J + R/LJ)/(S^2 + SR/L + KmKi/JL)

num_WTL=FdT_chen_w_TL.Numerator{1};
den_WTL=FdT_chen_w_TL.Denominator{1};

NORMALIZADO_WTL=den_WTL(1);  %Factor de normalizacion

% ****Normalizamos numerador y denominador****
num_n_WTL=num_WTL/NORMALIZADO_WTL;
den_n_WTL=den_WTL/NORMALIZADO_WTL;

FdT_WTL_n=tf(num_n_WTL,den_n_WTL)

W_WTL_n2=num_n_WTL(2);
W_WTL_n3=num_n_WTL(3);
W_WTL_d1=den_n_WTL(1);
W_WTL_d2=den_n_WTL(2);
W_WTL_d3=den_n_WTL(3);

J= (1/W_WTL_n2)
Ki=((W_WTL_d3*J*L)/Km)

%%
%%Seccion de Graficas%%

subplot(4,1,1)
plot(tiempo,w,'r'),hold on, plot(t__w_chen,simulada,'b'),title('Veloidad'); %,plot(t_chen,w_tl_chen,'g'), plot(t__w_chen,w_va_chen, 'b')


%Ploteamos los puntos marcados
plot(t1_va,yt1_va,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r',  'HandleVisibility','off')
plot(t2_va,y2t1_va,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r',  'HandleVisibility','off')
plot(t3_va, y3t1_va,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');

 plot(t_t1,y_t1,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g',  'HandleVisibility','off')
 plot(t_2t1,y_2t1,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g',  'HandleVisibility','off')
 plot(t_3t1,y_3t1,'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'g')
 legend('Curva real','Curva aproximada con Chen','Puntos de Va','Puntos de TL');


subplot(4,1,2)

plot(tiempo, I_armadura, 'b', 'DisplayName', 'Datos reales');
hold on;
plot(t_1, y_1, 'r', 'DisplayName', 'Modelo ajustado');
plot([t1i t2i t3i], [y1i y2i y3i]*uMAX, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Puntos medidos');

subplot(4,1,3)
plot(tiempo,tension),title('Tension');

subplot(4,1,4)
plot(tiempo,torque),title('Torque');


