clc
clear all;

%Universidad Nacional de Catamarca. Facultad de Tecnologia y Ciencias
%Aplicadas
%Alumno: Monroy Acosta Jose Ivan
%M.U:00881


% Ítem [4] Obtener el torque máximo que puede soportar el motor modelado mediante las Ecs. (1-5) 
% (1-6) y (1-7) cuando se lo alimenta con 12V, graficando para 5 segundos de tiempo la velocidad 
% angular y corriente ia para establecer su valor máximo como para dimensionar dispositivos 
% electrónicos. 

%%
%Definimos las variables

 L=366e-6; J=5e-9; R=55.6; Bm=0; Ki=6.49e-3; Km=6.53e-3;

t_euler=1e-7; %Tiempo de simulacion proporcionado por la catedra
tiempo=0: t_euler: 0.2;

tita_c= zeros(1,length(tiempo));
w_c= zeros(1,length(tiempo)); 
I_armadura_c = zeros(1,length(tiempo));
Torque = zeros(1, length(tiempo));

%Vector de variables de estado
X=[0;0;0];


%Definimos las matrices que representan el comportamiento del sistema

A=[(-R/L) 0 (-Km/L);0 0 1 ; (Ki/J) 0 (-Bm/J) ];
Bva=[1/L; 0; 0];
Btl=[0; 0; -1/J];


u = 12 ;           % Defnimos un vector u, de la longitud del tiempo de simulacion t, que valdra en c/componente 12

%%
for i = 2:length(tiempo)
    
    if(tiempo(i)>0.04 && tiempo(i)<0.07)
       Torque(i)=0.0014; 
    else
        Torque(i)=0;
        end
    %definimos nuestra vector de variables derivadas
  X_Punto=A*X+Bva*u+Btl*Torque(i);
  %Integramos para obtener X y definir el comportamiento del sistema
  X=X+t_euler*X_Punto ;
  
  %obtenemos los valores de las variables para cada iteracion
  I_armadura_c(i)=X(1);tita_c(i)=X(2);w_c(i)=X(3);
  %Torque(i) = Ki * X(1); % Torque electromagnético
  
 
end
%Obtenemos la corriente maxima, para con ello ver el torque maximo que
%soportara el motor

Tl_max=max(I_armadura_c)*Ki;

subplot(4,1,1);
plot(tiempo,w_c,'g'),title('Velocidad Angular');
subplot(4,1,2) ,plot(tiempo,I_armadura_c),title('Corrientede Armadura');
subplot(4,1,3) ,plot(tiempo,tita_c),title('Angulo');
subplot(4,1,4), plot(tiempo,Torque),title('Torque');