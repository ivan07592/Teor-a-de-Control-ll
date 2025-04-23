clear all;
clc;

%Definicion de los valores porpuestos por el ejercicio
R=47;
L=14e-6;
C=100e-9;


%Definicion de las matrices de estado
A=[(-R/L) (-1/L); 1/C 0];
B=[1/L; 0];
C=[R 0];
D=0;

%Defnicimos las Funcion de transfferencia:
[num,den]=ss2tf(A,B,C,D);
sys=tf(num,den)


h_euler=log(0.95)/(2*max(roots(den)))   % Determinamos el tiempo de integracion de Euler
Tfin = 0.003;                           %Tiempo maximo de simulacion
t = 0:h_euler:Tfin;                     %Armamos el vector de tiempo

pasos_por_mili = round(0.001 / h_euler);   % cantidad de pasos que hay en 1 en funcion del tiempo de Euler

%Definicion de los vectores del sistema
I = zeros(size(t));
V_resistencia = zeros(size(t));
V_cap = zeros(size(t));
X=[I; V_cap ];

% Generamos la señal u alternante
u = 12 * ones(size(t));           % Defnimos un vector u, de la longitud del tiempo de simulacion t, que valdra en c/componente 12
for i = 2:length(t)
    
    if mod(i, pasos_por_mili) == 0
        u(i) = -u(i-1);  % alterna el signo
    else
        u(i) = u(i-1);   % mantiene el valor anterior
    end
    
    %definimos nuestra vector de variables derivadas
  X_Punto=A*X+B*u(i);
  %Integramos para obtener X y definir el comportamiento del sistema
  X=X+h_euler*X_Punto ;
  
  %obtenemos los valores de las variables para cada iteracion
  I(i)=X(1);V_cap(i)=X(2);
  
  %obtenemos el valor de la tension en la resistencia
  V_resistencia(i)=R*I(i);
end

%  Simulamos la respuesta con lsim
% y = lsim(sys, u, t);
% figure;
subplot(4,1,1)
plot(t, u, 'b','LineWidth',1.5)
xlabel('Tiempo (s)'), ylabel('u(t) [V]')
title('Señal de entrada escalón de 12 V alternante')
grid on

subplot(4,1,2)
plot(t, V_cap, 'r','LineWidth',1.5)
xlabel('Tiempo (s)'), ylabel('V_cap')
title('Tension en el capacitor')
grid on

subplot(4,1,3)
plot(t, I, 'r','LineWidth',1.5)
xlabel('Tiempo (s)'), ylabel('I')
title('Corriente')
grid on

subplot(4,1,4)
plot(t, V_resistencia, 'r','LineWidth',1.5)
xlabel('Tiempo (s)'), ylabel('Vr')
title('Tension en la resistencia')
grid on

%De esta manera obtenemos la repsuesta del sistema, la cual ira dandonos
%picos de tension y corriente debido a que el capacitor actua como un
%cortocircuito hasta que se carga ppara que luego la tension en R caiga a
%cero cuando este se cargue, ya que la corriente en ese punto sera igual a
%cero y con ello VR=R*(0) igual a 0