clc
clear all;
%Definimos las variables

 L=366e-6; J=5e-9; R=55.6; Bm=0; Ki=6.49e-3; Km=6.53e-3;

% datos=xlsread("D:\Usuario\Documents\4°to AÑO\Teoria de Control ll\trabajos con Matlab 2025/Curvas_Medidas_Motor_2024.xlsx"); %Cargamos los datos proporcionados
% 
% vector=transpose(datos);
% filas=length(vector); %Obtenemos la cantidad de elementos

%Inicializamos los vectores, con la cantidad de filas que posee el archivo 
%transpuesto, y solo 
% w = zeros(1,filas); %entrada
% I_armadura = zeros(1,filas);
% tiempo=zeros(1,filas);

t_euler=1e-7; %Tiempo de simulacion proporccionado por la catedra
tiempo=0: t_euler: 0.2;

tita_c= zeros(1,length(tiempo));
w_c= zeros(1,length(tiempo)); 
I_armadura_c = zeros(1,length(tiempo));
Torque = zeros(1, length(tiempo));


%  X=[I_armadura_c; tita_c; w_c];
X=[0;0;0];


% Guardar cada fila en una variable diferente

% for i=1:1:3
%     info{i} = vector(i, :);
% end
% 
% % Guardo la informacion de la hoja de datos en sus vectores
% % correspondientes
% tiempo=info{1};
% w=info{2};
% I_armadura=info{3};


%Definimos las matrices que representan el comportamiento del sistema

A=[(-R/L) 0 (-Km/L);0 0 1 ; (Ki/J) 0 (-Bm/J) ];
Bva=[1/L; 0; 0];
Btl=[0; 0; -1/J];


u = 12 ;           % Defnimos un vector u, de la longitud del tiempo de simulacion t, que valdra en c/componente 12
%%
for j=1: length(tiempo)
    
end
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

Tl_max=max(I_armadura_c)*Ki;

subplot(4,1,1);
plot(tiempo,w_c,'g'),title('Velocidad Angular');
subplot(4,1,2) ,plot(tiempo,I_armadura_c),title('Corrientede Armadura');
subplot(4,1,3) ,plot(tiempo,tita_c),title('Angulo');
subplot(4,1,4), plot(tiempo,Torque),title('Torque');