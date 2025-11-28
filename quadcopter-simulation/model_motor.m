% Importa i dati (questo esempio presume che tu abbia già i vettori u e y)
u = motor_input;
y = motor_output;
Ts = 0.01;

% Crea un oggetto iddata
data = iddata(y, u, Ts);

% Visualizza i dati
figure;
subplot(2,1,1);
plot(u);
title('Segnale PWM (input)');
subplot(2,1,2);
plot(y);
title('Forza generata dall elica (output)');

% Rimuove il trend dai dati
data_detrended = detrend(data);

% Stima una funzione di trasferimento
np = 2;
nz = 1;
sys = tfest(data_detrended, np, nz);

% Visualizza la funzione di trasferimento stimata
disp(sys);

% Confronto tra il modello stimato e i dati
figure;
compare(data_detrended, sys);

