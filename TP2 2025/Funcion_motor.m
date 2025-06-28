function [X]=Funcion_motor(t_etapa, xant, accion) 
h=5e-6;

Laa=500e-6; J=2.5e-9; Ra=20; Bm=0;Ki=10e-3; Km=60.53e-3;%Motor inicial

% % Identificadas:
% Ra= 19.49908887350271; Laa=0.00046795934775014573; Ki= 0.009885263081304892;
% J= 1.5081203235402389e-09 ; Bm= 0.0; Km= 0.06053000001429517;
% Laa=366e-6;J=5e-9;Ra=55.6;Bm=0;Ki=6.49e-3;Km=6.53e-3;


x=xant;
ia=xant(1);wr=xant(2);titar=xant(3);Va=accion(1);TL=accion(2);
for ii=1:t_etapa/h
    ia_p=-(Ra/Laa)*ia-(Km/Laa)*wr+(1/Laa)*Va;
    wr_p=(Ki/J)*ia-(Bm/J)*wr-(1/J)*TL;
    ia=ia+ia_p*h;
    wr=wr+wr_p*h;
    titar=titar+wr*h;
end
X=[ia;wr;titar];
