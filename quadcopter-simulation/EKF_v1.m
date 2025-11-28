m = 1.0;  % Massa del quadricottero (kg)
g = [0; 0; -9.81];  % Accelerazione di gravità (m/s^2)
I = diag([0.005, 0.005, 0.01]);  % Tensore di inerzia (kg*m^2)

% x = [x, y, z, phi, theta, psi, vx, vy, vz, psi_dot, theta_dot, phi_dot]
n_states = 12;
x = zeros(n_states, 1);
% Parametri della simulazione
dt = 0.01; % Intervallo di tempo di campionamento (secondi)
T = 10; % Durata della simulazione (secondi)
time = 0:dt:T; % Vettore temporale

% Inizializzazione degli stati
x_true = zeros(n_states, 1); % Stato reale iniziale
x_est = zeros(n_states, 1); % Stato stimato iniziale
P_est = eye(n_states) * 0.1; % Covarianza iniziale
u = [1; 0; 0; 0]; % Controllo iniziale (thrust e momenti)

% Dati dei sensori simulati
gyro_data = zeros(3, length(time)); % Dati giroscopio simulati
acc_data = zeros(3, length(time)); % Dati accelerometro simulati

% Inizializzazione delle variabili per salvare i risultati
x_estimates = zeros(12, length(time)); % Stato stimato in ogni istante

% Simulazione del quadricottero
for k = 1:length(time)
    % Generazione di dati del sensore con rumore
    gyro_data(:, k) = [0.01; 0.02; 0.03] + randn(3, 1) * 0.01; % Rumore del giroscopio
    acc_data(:, k) = [0.0; 0.0; 9.81] + randn(3, 1) * 0.1; % Rumore dell'accelerometro

    % Pre-processing dei dati dei sensori
    if k == 1
        [z1, z2] = pre_proc(gyro_data(:, k), acc_data(:, k), x_est ,dt);
    else
        [z1, z2] = pre_proc(gyro_data(:, k), acc_data(:, k),x_estimates(:,k-1), dt);
    end

    % EKF update
    [x_est, P_est] = EKF(x_est, u, P_est, z1, z2, dt);

    % Salvataggio dei risultati
    x_estimates(:, k) = x_est;
end

% Traccia dei risultati
figure;
subplot(3,1,1);
plot(time, x_estimates(1, :), 'r', 'DisplayName', 'x stimato'); hold on;
plot(time, x_estimates(2, :), 'g', 'DisplayName', 'y stimato');
plot(time, x_estimates(3, :), 'b', 'DisplayName', 'z stimato');
xlabel('Tempo (s)');
ylabel('Posizione (m)');
legend;

subplot(3,1,2);
plot(time, x_estimates(4, :), 'r', 'DisplayName', 'Roll stimato'); hold on;
plot(time, x_estimates(5, :), 'g', 'DisplayName', 'Pitch stimato');
plot(time, x_estimates(6, :), 'b', 'DisplayName', 'Yaw stimato');
xlabel('Tempo (s)');
ylabel('Angoli (rad)');
legend;

subplot(3,1,3);
plot(time, x_estimates(7, :), 'r', 'DisplayName', 'Vx stimato'); hold on;
plot(time, x_estimates(8, :), 'g', 'DisplayName', 'Vy stimato');
plot(time, x_estimates(9, :), 'b', 'DisplayName', 'Vz stimato');
xlabel('Tempo (s)');
ylabel('Velocità (m/s)');
legend;



% EKF MAIN FUNCTION
function [x_upd, p_upd] = EKF(x, u, P, z1, z2, dt)
    [x_pred, p_pred] = predict(x, P, u, dt);
    [x_upd, p_upd] = update(x_pred, p_pred, z1, z2);
end

%EKF PREDICT
function [x_pred, p_pred] = predict(x, P, u, dt)
    F = computeJacobianF(x, u, dt);
    x_pred = f(x,u,dt);
    Q = eye(size(x)) * 0.01;
    p_pred = F * P * F' + Q;
end
%EKF UPDATE
function [x_upd, p_upd] = update(x_pred, p_pred, z1, z2)
    H1 = computeJacobianH(x);
    z_pred = h(x);
    % z1: misure fornite dall'integrazione dell'accelerometro per pos e vel
    % e dal giroscopio.
    %calcolo del primo termine di innovazione
    y1 = z1 - z_pred;
    %matrice di covarianza del rumore di misura
    R1 = eye(size(z1)) * 0.1;
    
    S1 = H1 * p_pred * H1' + R1;
    %compute first kalman gain 
    K1 = p_pred * H1' / S1;
    %first state update
    x1_upd = x_pred + K1 * y1;
    %covarince update
    p1_upd = (eye(length(x_pred)) - K1 * H1) * p_pred;
    
    %SECOND FASE OF UPDATE
    
    %seconda matrice di osservazione
    H2 = [[0,0,0,1,0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0,0,0,0,0]];
    %calcolo del secondo termine di innovazione
    y2 = z2 - z_pred(4:5);
    %matrice di covarianza del rumore di misura
    R2 = eye(size(z2)) * 0.1;
    
    S2 = H2 * p1_upd * H2' + R2;
    %second kalman gain
    K2 = p1_upd * H2' / S2;
    %second state update
    x_upd = x1_upd + K2 * y2;
    %covarince update
    p_upd = (eye(length(x_pred)) - K2 * H2) * p1_upd;
    
end

%quadcopter model
function x_predict = f(x,u,dt)
    p = x(1:3);
    euler = x(4:6);
    v = x(7:9);
    euler_dot = x(10:12);
    
    thrust = u(1);
    M = u(2:4);
    %delta position
    dp = v * dt;
    %delta euler angles
    deuler = euler_dot * dt;
    %delta velocity
    dv = (1/m) * (body_to_earth(euler) * [0; 0; thrust] - [0; 0; m * g]) * dt;
    %delta euler angles dot
    body_rates = euler_rate_to_body_rate(euler(1), euler(2), euler_dot);
    d_body_rates  = inv(I)* (M - cross(body_rates , I * body_rates)) * dt;
    
    
    x_predict = zeros(size(x));
    x_predict(1:3) = p + dp;
    x_predict(4:6) = euler + deuler;
    x_predict(7:9) = v + dv;
    updated_body_rates = body_rates + d_body_rates;
    x_predict(10:12) = body_rate_to_euler_rate(euler(1), euler(2), updated_body_rates(1), updated_body_rates(2), updated_body_rates(3));
end
%sensor model
function z_pred = h(x)
    T = eye(n_states);
    z_pred = T * x;
end
% jacobian of h(x)
function H = computeJacobianH(x)
    % La Jacobiana della funzione di osservazione h(x)
    % è semplicemente la matrice identità di dimensione n_states x n_states
    H = eye(n_states);
end
% jacobian of f(x)
function F = computeJacobianF(x, u, dt)
    % Estrarre stati dal vettore x
    p = x(1:3);          % Posizione
    euler = x(4:6);      % Angoli di Eulero: roll, pitch, yaw
    v = x(7:9);          % Velocità lineari
    euler_dot = x(10:12);% Velocità angolari
    
    thrust = u(1);       % Spinta
    M = u(2:4);          % Momenti

    % Matrice di rotazione dal body frame al earth frame
    R_b_to_e = body_to_earth(euler); % Funzione da definire

    % Derivata della matrice di rotazione rispetto agli angoli di Eulero
    dRdeuler = dR_body_to_earth(euler); % Funzione da definire

    % Derivata parziale della velocità lineare rispetto agli angoli di Eulero
    dv_deuler = (1/m) * dRdeuler * [0; 0; thrust] * dt;

    % Derivate parziali per le velocità angolari
    dB_deuler = dBody_rate_to_euler_rate(euler); % Funzione da definire

    % Costruzione della Jacobiana
    F = zeros(12, 12);

    % Derivate rispetto alla posizione p
    F(1:3, 1:3) = eye(3);           % d(p + dp)/dp
    F(1:3, 7:9) = dt * eye(3);      % d(p + dp)/dv

    % Derivate rispetto agli angoli di Eulero
    F(4:6, 4:6) = eye(3);           % d(euler + deuler)/deuler
    F(4:6, 10:12) = dt * eye(3);    % d(euler + deuler)/deuler_dot

    % Derivate rispetto alla velocità v
    F(7:9, 7:9) = eye(3);           % d(v + dv)/dv
    F(7:9, 4:6) = dv_deuler;        % d(v + dv)/deuler

    % Derivate rispetto alle velocità angolari euler_dot
    F(10:12, 4:6) = dB_deuler;      % d(body_rate_to_euler_rate)/deuler
    F(10:12, 10:12) = eye(3);       % d(body_rate_to_euler_rate)/deuler_dot
end


%PRE-PROCESSING FOR SENSORS MEASURMENTS
function [z1, z2] = pre_proc(gyro, acc, x_prev, dt)
    %NOTE: capire se usare per integrare la misura precedente o gli stati
    %stimati dall'EKF
    %NOTE 2: aggiunta di un low-pass filter per le misurazioni 
    
    phi = x_prev(4);
    theta = x_prev(5);
    %resolving accelerations in intertial frame without gravity
    acc_e = body_to_earth(euler) * (acc - earth_to_body(euler) * g);
    %from body rate measured by gyro to euler rate
    z1(10:12) = body_rate_to_euler_rate(phi, theta, gyro(1), gyro(2), gyro(3));
    %integrate gyro for euler angles
    z1(4:6) = x_prev(4:6) + z1(10:12) * dt;
    %integrate accelerations to update velocity
    z1(7:9) = x_prev(7:9) + acc_e * dt;
    %
    z1(1:3) = x_prev(1:3) + z1(7:9) * dt;
    
    a_x = acc(1);
    a_y = acc(2);
    a_z = acc(3);
    
    % Calcolo degli angoli di roll e pitch attraverso l'accelerometro
    
    z2(1) = atan2(a_y, a_z); % roll (φ)
    z2(2) = atan2(-a_x, sqrt(a_y^2 + a_z^2)); % pitch (θ)
end

% Funzione per calcolare la derivata parziale della matrice di rotazione rispetto agli angoli di Eulero
function dRdeuler = dR_body_to_earth(euler)
    phi = euler(1);   % Roll
    theta = euler(2); % Pitch
    psi = euler(3);   % Yaw
    
    % Calcolo delle derivate parziali
    dR_dphi = [0, 0, 0; 
               cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi), cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi), cos(phi) * cos(theta);
               -sin(phi) * sin(theta) * cos(psi) + cos(phi) * sin(psi), -sin(phi) * sin(theta) * sin(psi) - cos(phi) * cos(psi), -sin(phi) * cos(theta)];

    dR_dtheta = [-sin(theta) * cos(psi), -sin(theta) * sin(psi), -cos(theta);
                 sin(phi) * cos(theta) * cos(psi), sin(phi) * cos(theta) * sin(psi), -sin(phi) * sin(theta);
                 cos(phi) * cos(theta) * cos(psi), cos(phi) * cos(theta) * sin(psi), -cos(phi) * sin(theta)];
             
    dR_dpsi = [-cos(theta) * sin(psi), cos(theta) * cos(psi), 0;
               -sin(phi) * sin(theta) * sin(psi) - cos(phi) * cos(psi), sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi), 0;
               -cos(phi) * sin(theta) * sin(psi) + sin(phi) * cos(psi), cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi), 0];
    
    % Concatenazione delle derivate parziali lungo la terza dimensione
    dRdeuler = cat(3, dR_dphi, dR_dtheta, dR_dpsi);
end

% Funzione per calcolare la derivata parziale della trasformazione da velocità angolari body a Euler rate
function dB_deuler = dBody_rate_to_euler_rate(euler)
    phi = euler(1);   % Roll
    theta = euler(2); % Pitch
    
    % Derivate parziali della matrice di trasformazione
    dT_dphi = [0, cos(phi) * tan(theta), -sin(phi) * tan(theta);
               0, -sin(phi), -cos(phi);
               0, cos(phi)/cos(theta), -sin(phi)/cos(theta)];
           
    dT_dtheta = [0, sin(phi) * sec(theta)^2, cos(phi) * sec(theta)^2;
                 0, 0, 0;
                 0, sin(phi) * sin(theta) / cos(theta)^2, cos(phi) * sin(theta) / cos(theta)^2];
    
    % Concatenazione delle derivate parziali
    dB_deuler = cat(3, dT_dphi, dT_dtheta);
end

%rotations matrices
function R_be = body_to_earth(euler)
    % body_to_earth: Calcola la matrice di rotazione dal sistema di coordinate del corpo (body frame) 
    %                al sistema di coordinate della Terra (earth frame) usando gli angoli di Euler.

    % Input:
    %   phi   - Roll angle (rad)
    %   theta - Pitch angle (rad)
    %   psi   - Yaw angle (rad)
    phi = euler(1);
    theta = euler(2);
    psi= euler(3);
    % Calcola le matrici di rotazione attorno agli assi Z, Y e X
    Rz = [cos(psi), -sin(psi), 0;
          sin(psi), cos(psi), 0;
          0, 0, 1];
      
    Ry = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];
      
    Rx = [1, 0, 0;
          0, cos(phi), -sin(phi);
          0, sin(phi), cos(phi)];
      
    % Matrice di rotazione totale dal sistema Body a Earth
    R_be = Rz * Ry * Rx;
end
function R_eb = earth_to_body(euler)
    % earth_to_body: Calcola la matrice di rotazione dal sistema di coordinate della Terra (earth frame) 
    %                al sistema di coordinate del corpo (body frame) usando gli angoli di Euler.

    % Input:
    %   phi   - Roll angle (rad)
    %   theta - Pitch angle (rad)
    %   psi   - Yaw angle (rad)
    phi = euler(1);
    theta = euler(2);
    psi= euler(3);
    
    % Matrice di rotazione da body a earth
    R_be = body_to_earth(phi, theta, psi);
    
    % Matrice di rotazione da earth a body (inversa di R_be)
    R_eb = R_be';
end
function euler_rate = body_rate_to_euler_rate(phi, theta, p, q, r)

    % body_rate_to_euler_rate: Converte le velocità angolari del corpo (body rates) 
    %                          in tassi di variazione degli angoli di Euler (Euler rates).
    
    % Input:
    %   phi   - Roll angle (rad)
    %   theta - Pitch angle (rad)
    %   p, q, r - Velocità angolari nel frame del corpo (body rates) (rad/s)

    % Matrice di conversione da body rate a Euler rate
    T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi), -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    
    % Calcolo degli Euler rates
    euler_rate = T * [p; q; r];
end
function body_rate = euler_rate_to_body_rate(phi, theta, euler_rate)
    % euler_rate_to_body_rate: Converte i tassi di variazione degli angoli di Euler (Euler rates)
    %                          in velocità angolari del corpo (body rates).
    
    % Input:
    %   phi, theta - Angoli di rollio (phi) e beccheggio (theta) (rad)
    %   euler_rate - Vettore dei tassi di variazione degli angoli di Euler [dphi, dtheta, dpsi] (rad/s)

    % Matrice di conversione da Euler rate a body rate (inversa di T)
    T_inv = [1, 0, -sin(theta);
             0, cos(phi), sin(phi)*cos(theta);
             0, -sin(phi), cos(phi)*cos(theta)];

    % Calcolo dei body rates
    body_rate = T_inv * euler_rate;
end