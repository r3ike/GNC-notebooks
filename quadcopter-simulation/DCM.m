% Angoli di Eulero (in radianti)
phi = deg2rad(30);    % Roll
theta = deg2rad(20);  % Pitch
psi = deg2rad(10);    % Yaw

% Matrice di rotazione per l'asse x (roll)
R_x = [1 0 0;
       0 cos(phi) sin(phi);
       0 -sin(phi) cos(phi)];

% Matrice di rotazione per l'asse y (pitch)
R_y = [cos(theta) 0 -sin(theta);
       0 1 0;
       sin(theta) 0 cos(theta)];

% Matrice di rotazione per l'asse z (yaw)
R_z = [cos(psi) sin(psi) 0;
       -sin(psi) cos(psi) 0;
       0 0 1];

% Matrice di rotazione da Earth a Body
R_eb = R_z * R_y * R_x;

% Matrice di rotazione da Body a Earth (trasposta di R_eb)
R_be = R_eb';

% Visualizza la matrice DCM
disp('Matrice DCM da Body a Earth (R_be):');
disp(R_be);

% Esempio di vettore nel sistema di riferimento del corpo (body frame)
v_body = [1; 2; 3];

% Trasforma il vettore nel sistema di riferimento terrestre (earth frame)
v_earth = R_be * v_body;

% Visualizza il risultato
disp('Vettore nel sistema di riferimento Earth (v_earth):');
disp(v_earth);
