clear all; 
clc;

%Universidad Nacional de Catamarca. Facultad de Tecnologia y Ciencias
%Aplicadas
%Alumno: Monroy Acosta Jose Ivan
%M.U:00881

% Ítem [6] Implementar un PID en tiempo discreto para que el ángulo del motor permanezca en una 
% referencia de 1radian sometido al torque descripto en la Fig. 1-3. (Tip: partir de KP=0,1; 
% Ki=0,01; KD=5). 

%%
%DEFINICION DE VARIABLES Y PARAMETROS
X=-[0;0;0];ii=0;t_etapa=1e-5;titaRef=1;tF=10;

%Constantes del PID 

KP=15;KI=150;KD=0.000005;
color_='r'; 

Ts=t_etapa; 

t=0:t_etapa:tF;
%%

torque=zeros(1,length(t));
x1=zeros(1,length(t));
x2=zeros(1,length(t));
x3=zeros(1,length(t));
acc=zeros(1,length(t));

%Formulas extraidas de los apuntes. Con ellas definimos las constantes del
%PID

A1=((2*KP*Ts)+(KI*Ts^2)+(2*KD))/(2*Ts);
B1=((-2*KP*Ts)+(KI*Ts^2)-(4*KD))/(2*Ts);
C1=KD/Ts;

%Vector de error
e=zeros(round(tF/t_etapa),1);u=0; 



%%
%ETAPA DE CONTROL DEL MOTOR


for ii = 1:length(t) 
   
    k=ii+2;
    %Defiimos un torque igual a cero para los instantes no deseados
    
    TL=0;
    
    if(t(ii)>4 && t(ii)<7)
       TL=0.0014;
    end
    
    %Enviamos los datos a nuestra funcion para que me devuelva las
    %variables de control en nuestro vector X
    X=Funcion_motor(t_etapa, X, u, TL); 
    e(k)=titaRef-X(1); %ERROR 
    
    u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
    
   if u>12         %limito accion de control a +-12
        u=12;
    end
    if u<-12
        u=-12;
    end
    
    x1(ii)=X(1);%tita
    x2(ii)=X(2);%wp 
    x3(ii)=X(3);
    acc(ii)=u; 
    torque(ii)=TL;
end 
 
%%
%ETAPA DE SIMULACION

 subplot(5,1,1);hold on; 
plot(t,x1 ,color_);title('Posicion Angular'); yline(1,  'g--', 'tita ref=1',  'HandleVisibility','off');

subplot(5,1,2);hold on; 
plot(t,acc,color_);title('Accion de Control'); 
xlabel('Tiempo [Seg.]');

subplot(5,1,3); hold on;
plot(t,x3); title('corriente')

subplot(5,1,4)
plot(t,x2,'r');title('wr')

subplot(5,1,5)
plot(t,torque); title('Carga')