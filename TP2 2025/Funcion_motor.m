function [X]=Funcion_motor(t_etapa, xant, accion,TL) 
Laa=366e-6; Jm=5e-9;Ra=55.6;Bm=0;Ki=6.49e-3;Kb=6.53e-3; 
Va=accion; 
h=1e-5; 
titar= xant(1); 
wr= xant(2); 
ia=xant(3);
for ii=1:t_etapa/h    
    
ia_p = -(Ra / Laa) * ia - (Kb / Laa) * wr + (1 / Laa) * Va;
wr_p = (Ki / Jm) * ia - (Bm / Jm) * wr - (1 / Jm) * TL;

ia = ia + ia_p * h;
wr = wr + wr_p * h;
titar = titar + wr * h;

end 
X=[titar; wr;ia ]; 
