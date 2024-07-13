clc; close all; clear

Ma = ureal('Ma', -300.4211, 'Range', [-472.271 -126.249]);
Md = ureal('Md', -131.3944, 'Range', [-163.582 -98.149]);

% Load data 
load('Robust/Git/sensitivity_analysis_data.mat', 'g', 'a', 'M', 'Za', 'Mq', 'Zd', 'Aa', 'Ad', 'omega_a', 'zeta_a');

% State Space of the linear model of the missile airframe short period pitch axis
A = [-Za/(M*a) 1; Ma Mq]; B = [-Zd/(M*a); Md]; 
C = [-Aa/g 0; 0 1]     ; D = [-Ad/g; 0];

G_m = ss(A, B, C, D);
G_m.InputName  = {'u_m'};
G_m.StateName  = {'x1', 'x2'};
G_m.OutputName = {'y1', 'y2'};

% State space of the actuator
A_act = [0 1; -omega_a^2 -2*zeta_a*omega_a];
B_act = [0; omega_a^2];
C_act = [1 0; 0 1]; D_act = [0; 0];

G_a = ss(A_act, B_act, C_act, D_act);
G_a.InputName  = {'u_cmd'};
G_a.StateName  = {'x3', 'x4'};
G_a.OutputName = {'u_m', 'udot_m'};

% Load Simulink file
load_system('Airframe');

% Perform linearization 
io(1) = linio('Airframe/u_cmd', 1, 'input');
io(2) = linio('Airframe/Missile', 1, 'output');
G_am = linearize('Airframe', io);                               % states are      x3, x4, x1, x2
G_am = ss2ss(G_am, [[0 0 1 0];[0 0 0 1];[1 0 0 0 ];[0 1 0 0]]); % shift states to x1, x2, x3, x4
G_am.InputName = {'u_{cmd}'};
G_am.OutputName = {'y1', 'y2'};

iopzmap(G_am)
save('G_m.mat', 'G_m')
save('G_a.mat', 'G_a')
save('G_am.mat', 'G_am')

%% Part 2: Loop Shaping - Damping Gain Design (5%)

% open loop pitch rate
G_ol_q = G_am(2,1);

%rlocus(-G_ol_q)

% Damping gain obtained from root locus for damping = 0.7
C_q = -0.163;

load_system('ClosedLoop_Cq.slx');

% Linearize model
io(1) = linio('ClosedLoop_Cq/u_unsc', 1, 'input');
io(2) = linio('ClosedLoop_Cq/Demux2', 1, 'output');
G_cl_q_unsc = linearize('ClosedLoop_Cq', io);
G_cl_q_unsc = ss2ss(G_cl_q_unsc, [0 0 1 0; 0 0 0 1; 1 0 0 0; 0 1 0 0]); % shift states
G_cl_q_unsc.InputName = {'u_{unsc}'};
G_cl_q_unsc.OutputName = {'y1'};

save('C_q.mat', 'C_q') 

%% Scaling Gain Design (5%)
clc 

% Compute the Scaling Gain Csc
dc_gain = dcgain(G_cl_q_unsc);
C_sc = 1 / dc_gain;

load_system('ClosedLoop_CqCsc.slx');

% Linearize the model
io(1) = linio('ClosedLoop_CqCsc/u_p', 1, 'input');
io(2) = linio('ClosedLoop_CqCsc/Demux2', 1, 'output');
G = linearize('ClosedLoop_CqCsc', io);
G = ss2ss(G, [[0 0 1 0];[0 0 0 1];[1 0 0 0 ];[0 1 0 0]]); 
G.InputName = {'u_{p}'};
G.OutputName = {'y1'};

%Total pre-compensated inner loop transfer function G
G_tf = tf(G);
%Confirm it converges to 1
step(G_tf)

save('C_sc.mat', 'C_sc') 
save('G.mat', 'G') 


%% Integral Gain Design (10%) sisotool

% Load Simulink model
load_system('ClosedLoop_CqCscCi');

C_i = 1;

% Linearize the model
io(1) = linio('ClosedLoop_CqCscCi/e1_f', 1, 'input');
io(2) = linio('ClosedLoop_CqCscCi/Demux2', 1, 'output');
G_ol_nz = linearize('ClosedLoop_CqCscCi', io);
G_ol_nz = ss2ss(G_ol_nz, [[0 0 1 0 0];[0 0 0 1 0];[1 0 0 0 0];[0 1 0 0 0]; [0 0 0 0 1]]); % shift states
G_ol_nz.InputName = {'e1_{f}'};
G_ol_nz.OutputName = {'y1'};

% sisotool(G_ol_nz)
C_i_pm = 6.2658; % sisotool gives a C_i value of 6.2658 to guarantee 60 degrees PM

save('C_i_pm.mat', 'C_i_pm')

%% Integral Gain Design (10%) settling time C_i_st

% % Define the initial value of C_i and the increment
% C_i_initial = 5;
% C_i_increment = 0.1;
% C_i_max = 7; % You can set a maximum value for C_i to limit the search
% yfinal = 1; % Steady-state response value
% 
% C_i_st = C_i_initial;
% min_settling_time = inf;

% % Loop through values of C_i
% for C_i = C_i_initial:C_i_increment:C_i_max
% 
%     % Linearize the system
%     io(1) = linio('ClosedLoop_CqCscCi/e1_f', 1, 'input');
%     io(2) = linio('ClosedLoop_CqCscCi/Demux2', 1, 'output');
%     G_ol_nz = linearize('ClosedLoop_CqCscCi', io);
%     G_ol_nz = ss2ss(G_ol_nz, [[0 0 1 0 0];[0 0 0 1 0];[1 0 0 0 0];[0 1 0 0 0]; [0 0 0 0 1]]); % shift states
% 
%     % Get the step response and calculate the settling time
%     [y, t] = step(G_ol_nz/(G_ol_nz+1));
%     S = stepinfo(y, t, yfinal, 'SettlingTimeThreshold', 5/100);
%     Ts = S.SettlingTime;
% 
%     % Check if this is the smallest settling time found
%     if Ts < min_settling_time
%         min_settling_time = Ts;
%         C_i_st = C_i;
%     end
% end

C_i_st = 5.9; % Value obtained by iteration

% Display the optimal C_i and the corresponding settling time
fprintf('Optimal C_i: %.2f\n', C_i_st);
fprintf('Minimum 5%% Settling Time: %.2f seconds\n', 0.27);
fprintf('Phase Margin: %.2f degrees\n', 61.9);

save('C_i_st', 'C_i_st')

%clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm C_i_st
%% Ploting
s = zpk('s');

T1 = feedback(G * C_i_pm / s, 1, -1);
T2 = feedback(G * C_i_st / s, 1, -1);

figure;
step(G)
hold on;
step(T1)
step(T2)
grid on;
%xline('--', 'Settling time', 'LabelVerticalAlignment', 'bottom');
yline(1.05, '--', '+5% threshold');
yline(0.95, '--', '-5% threshold');
xlabel('Time (seconds)');
ylabel('Response');
title('Step Response with Optimal C_i');
legend('Step Response (G)', 'sisotool', 'Settling Time', '+5% Threshold', '-5% Threshold');

%clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm C_i_st
%% Part #3a – Weighting filters (2.5%)
close all
clc
% Weighting filters
%%%%%%%%%%%%%%%%%%%

% Low-pass filter parameters W_1
M = 1.5;                  % high gain at low frequencies while also ensuring PM above 30 degrees
A = 0.001;                % zero gain at high frequencies
mag_W_1 = db2mag(3.01);   % -3.01 dB at 4 rad/s
freq_W_1 = 4.0;           % [rad/s]

% High-pass filter parameters W_2, assume M_1 = A_2 , A_1 = M_2
mag_W_2 = db2mag(15);     % -15 dB at -3.01dB bandwidth frequency
freq_W_2 = 151;           % freq where magnitude of actuator dynamics equals -3.01dB [rad/s]

S_t     = tf([1 freq_W_1*A],[1/M freq_W_1]);
W_1     = ss(1/S_t);
W_1_tf  = tf(W_1);
KS_t     = tf([A freq_W_2],[1 freq_W_2/M]);
W_2     = ss(1/KS_t);
W_2_tf  = tf(W_2);

figure;
sigma(1/W_1, 'r', 1/W_2, 'b')
grid on;
legend('1/W1', '1/W2');
title('Inverse Gain of Weighting Filters');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');

W_1 = tf(makeweight(1/A, [freq_W_1, mag_W_1], 1/M));
W_2 = tf(makeweight(1/M, [freq_W_2, mag_W_2], 1/A));

PM = rad2deg(2*asin(1/(2*M))); %Yields a PM of 38.94 degrees


%Plot the filters' inverse gain
figure;
sigma(1/W_1, 'r', 1/W_2, 'b')
grid on;
legend('1/W1', '1/W2');
title('Inverse Gain of Weighting Filters');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');

figure;
bode(W_1_tf, 'r', W_1, 'b')

figure;
bode(W_2_tf, 'r', W_2, 'b')

% Save the filters for later use
save('W1.mat', 'W_1');
save('W2.mat', 'W_2');


%% Part #3B.1 – Reference model computation (5%)

% Find non-minimum phase zero for G_am
% iopzmap(G_am)
z_m = 36.6; 

% Target parameters
ts_d = 0.18;           % Settling time (s)
Md = 5;                % Overshoot %

% upper bound guesses for omega_d (half of z_m) and zeta_d (1)
ub = [z_m/2, 1]; % maximum achievable bandwidth is approximately equal to half of that of the NMP zero of the open loop acceleration channel.

% random initial guess
x0 = [40, 0.8];

% Define the cost function for optimization
cost_function = @(x) ...
(stepinfo(tf([-x(1)^2/z_m, x(1)^2], [1, 2*x(2)*x(1), x(1)^2]),'SettlingTimeThreshold', 5/100).SettlingTime - ts_d)^2 + ...
(stepinfo(tf([-x(1)^2/z_m, x(1)^2], [1, 2*x(2)*x(1), x(1)^2]),'SettlingTimeThreshold', 5/100).Overshoot - Md)^2;

% Use fmincon to find optimal omega_d and zeta_d
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
[x_opt, ~] = fmincon(cost_function, x0,[], [], [], [], [], ub, [], options);

omega_d = x_opt(1);
zeta_d = x_opt(2);

% Construct the transfer function T_d
T_d = tf([-omega_d^2 / z_m omega_d^2], [1, 2 * zeta_d * omega_d, omega_d^2]); % Reference model% Plot the step response and show the characteristics
figure;
step(T_d);
grid on;
title('Step Response of the Reference Model T_d');
xlabel('Time (s)');
ylabel('Response');

% Display characteristics on the plot
S = stepinfo(T_d, 'SettlingTimeThreshold', 5/100);
hold on;
yline(1 + Md/100, '--', '5% Overshoot');
yline(1 - Md/100, '--', '5% Undershoot');
xline(S.SettlingTime, '--', 'Settling Time');

% Save the reference model
save('T_d.mat', 'T_d');

%clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm C_i_st W_1_tf W_2_tf W_1 W_2 T_d


%% Part #3c - Feedback controller design (hinfsyn case)
clc 
clear io
% Low-pass filter parameters W_3
W_3 = W_1; % Set W3 equal to W1 for relaxed model following constraint

% Either linearize or use the proof of exercise 2, both give obviously the same answer
% Open the Simulink model
load_system('Design');

% Specify the input and output points for linearization
io(1) = linio('Design/w', 1, 'input');
io(2) = linio('Design/u', 1, 'input');
io(3) = linio('Design/W_1', 1, 'output');
io(4) = linio('Design/W_2', 1, 'output');
io(5) = linio('Design/W_3', 1, 'output');
io(6) = linio('Design/Sum2', 1, 'output');

save('io_Design.mat', 'io')

% Linearize the model
P = linearize('Design', io);   % Here the uncertainties are neglected
P.InputName = {'w', 'u'};
P.OutputName = {'w1', 'w2', 'w3', 'v'};

save('P.mat', 'P');

%% 3C.1 Exercise 3

% Define the number of measurements and controls
nmeas = 1; % Number of measurements (outputs to the controller)
nctrl = 1; % Number of controls (inputs from the controller)

% Set optimization options for hinfsyn
opts = hinfsynOptions('RelTol', 1e-6);

% Design the H-infinity controller using hinfsyn
[C_e, T_wz, gamma] = hinfsyn(P, nmeas, nctrl, opts);
gamma
save('C_e.mat', 'C_e')

% Compute the closed-loop transfer function with weights
S_o = 1/(1+G*C_e);
T_o = 1 - S_o;
%T_wz = [W_1 * S_o; W_2 * C_e * S_o; W_3 * (T_d - T_o)]

% Define the frequency range
freq_range = {1, 300}; % Frequency range from 1 to 300 rad/s

% Plot the singular values of T_wz and its individual components
figure;
sigma(T_wz, freq_range);
hold on;
sigma(T_wz(1,:), freq_range, 'r--');
sigma(T_wz(2,:), freq_range, 'g--');
sigma(T_wz(3,:), freq_range, 'b--');
legend('\sigma(T_{wz})', '\sigma(T_{wz1})', '\sigma(T_{wz2})', '\sigma(T_{wz3})');
title('Singular Value Plot');
xlabel('Frequency (rad/s)');
ylabel('Singular Value (Amplitude)');
hold off;

% Compute norms for individual transfer functions
gamma_1 = norm(T_wz(1,:), 'inf');
gamma_2 = norm(T_wz(2,:), 'inf');
gamma_3 = norm(T_wz(3,:), 'inf');

% Display individual performance levels
disp('Individual Performance Levels gamma_i:');
disp(['gamma_1: ', num2str(gamma_1)]);
disp(['gamma_2: ', num2str(gamma_2)]);
disp(['gamma_3: ', num2str(gamma_3)]);

% Verify the global performance level
norm_T_wz_inf = norm(T_wz, 'inf');
disp('Global Performance Level (norm of T_wz):');
disp(norm_T_wz_inf);

%% 3C.1 Exercise 4
clear io 
close all
% Tune magnitude for parameter W_3
mag_W_3 = db2mag(8.25);  % at -8.25 dB gamma_1 reaches exactly 1

W_3 = tf(makeweight(1/A, [freq_W_1, mag_W_3], 1/M));

save('W3.mat', 'W_3')

load_system('Design');
io = load('io_Design.mat');

% Linearize the model
P = linearize('Design', io); 
P.InputName = {'w', 'u'};
P.OutputName = {'w1', 'w2', 'w3', 'v'};

% Design the H-infinity controller using hinfsyn
[C_e, T_wz, gamma] = hinfsyn(P, nmeas, nctrl, opts);

% Compute the closed-loop transfer function with weights
S_o = 1/(1+G*C_e);
T_o = 1 - S_o;
freq_range = {1, 300}; % Frequency range from 1 to 300 rad/s

% Plot the singular values of T_wz and its individual components
figure;
sigma(T_wz, freq_range);
hold on;
sigma(T_wz(1,:), freq_range, 'r--');
sigma(T_wz(2,:), freq_range, 'g--');
sigma(T_wz(3,:), freq_range, 'b--');
legend('\sigma(T_{wz})', '\sigma(T_{wz1})', '\sigma(T_{wz2})', '\sigma(T_{wz3})');
title('Singular Value Plot');
xlabel('Frequency (rad/s)');
ylabel('Singular Value (Amplitude)');
hold off;

% Compute norms for individual transfer functions
gamma_1 = norm(T_wz(1,:), 'inf');
gamma_2 = norm(T_wz(2,:), 'inf');
gamma_3 = norm(T_wz(3,:), 'inf');

% Display individual performance levels
disp('Individual Performance Levels gamma_i:');
disp(['gamma_1: ', num2str(gamma_1)]);
disp(['gamma_2: ', num2str(gamma_2)]);
disp(['gamma_3: ', num2str(gamma_3)]);

% Verify the global performance level
norm_T_wz_inf = norm(T_wz, 'inf');
disp('Global Performance Level (norm of T_wz):');
disp(norm_T_wz_inf);


%% 3C.2 Controller order reduction (5%)
% Load the initial controller C0_e (example given, replace with actual data) Assuming C0_e is a transfer function object
% load('C0_e.mat')
close all; clc
%pzmap(minreal(C0_e))
F_f = 1;
C0_e = C_e;

disp('Initial Controller C_e: ')
zpk(C0_e)
[z, p, k] = zpkdata(C0_e,'v');

% Extract poles and zeros from the transfer function C0_e
original_poles = pole(C0_e);
original_zeros = zero(C0_e);

% Identify poles and zeros to keep
poles_to_keep = original_poles([false true true true true true true true false]);
zeros_to_keep = original_zeros([false true true true true true true false]);

% Create the new transfer function with the kept poles and zeros
C_e_min = zpk(zeros_to_keep, poles_to_keep, db2mag(30));
%C_i_min = minreal(tf(C_e_min * tf([1 0], 1)));
poles_to_keep = original_poles([false true true true true true true false false]);
C_i_min = zpk(zeros_to_keep, poles_to_keep, db2mag(30));

R = reducespec(C_i_min,"balanced");
figure;
view(R)
C_i_red = getrom(R,Order=2);

disp('Minimum Integral Controller C_i_min: ')
zpk(C_i_min)

disp('Reduced Integral Controller C_i_red: ')
zpk(C_i_red)

figure;
freq_range = {1, 100000};
bode(C0_e, 'r', C_e_min, 'b--', freq_range);
legend("Original", "Order 7");
grid on;
legend('C0_e', 'C_e_min');
title('Bode Plot of Full and Simplified Controllers');

figure;
range = {1, 1000};
bode(C_i_min, 'r', C_i_red, 'b--', range);
legend('C_i_min', 'C_i_red');
title('Bode Plot of Integral and Reduced Controllers');

% Plot pole-zero maps
figure;
pzmap(C_i_min);
title('Pole-Zero Map of C_i_min');
figure;
pzmap(C_i_red);
title('Pole-Zero Map of C_i_red');

% Save controllers if needed
save('C_e_min.mat', 'C_e_min');
save('C_i_red.mat', 'C_i_red');


%% Part #3C.3 - Controller analysis & simulation (5%)
close all;

% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);

C_i = C_i_red;

% Define the I/O points for linearization
io(1) = linio([model '/r'], 1, 'input');   % Input r
io(2) = linio([model '/d_i'], 1, 'input'); % Input d_i
io(3) = linio([model '/Sum3'], 1, 'output'); % Output e1
io(4) = linio([model '/C_i'], 1, 'output');  % Output u
io(5) = linio([model '/Demux1'], 1, 'output'); % Output y1
io(6) = linio([model '/Sum1'], 1, 'output'); % Output e1_d
io(7) = linio([model '/Sum2'], 1, 'output'); % Output udot_m
io(8) = linio([model '/Demux'], 2, 'output'); % Output u_p

save('io_ClosedLoop_Test.mat', 'io')

% Linearize the model
T = linearize(model, io);

% Extract required transfer functions
T_r_to_e1 = T(1,1);   % T(1,1) = S_o
T_r_to_u = T(2,1);    % T(2,1) = CeS_o
T_r_to_y1 = T(3,1);   % T(3,1) = T_o
T_r_to_e1_d = T(4,1); % T(4,1) = T_m
T_r_to_udot_m = T(6,1); % T(6,1)
T_di_to_u = T(2,2);   % T(2,2) = -T_i
T_di_to_y1 = T(3,2);  % T(3,2) = SoG
T_di_to_up = T(5,2);  % T(5,2) = S_i

% Define the weighting filter inverses for comparison
W1_inv = inv(W_1);
W2_inv = inv(W_2);
W3_inv = inv(W_3);

% Plot singular values
figure;

subplot(2,3,1);
sigma(W1_inv, 'r', T_r_to_e1, 'b', T_di_to_up, 'g');
title('Singular Values: W1^{-1}, S_o, S_i');
legend('W1^{-1}', 'S_o', 'S_i');

subplot(2,3,2);
sigma(W2_inv, 'r', T_r_to_u, 'b');
title('Singular Values: W2^{-1}, CeS_o');
legend('W2^{-1}', 'CeS_o');

subplot(2,3,3);
sigma(W3_inv, 'r', T_r_to_e1_d, 'b');
title('Singular Values: W3^{-1}, T_m');
legend('W3^{-1}', 'T_m');

subplot(2,3,4);
sigma(-T_di_to_u, 'b', T_r_to_y1, 'g');
title('Singular Values: T_i, T_o');
legend('T_i', 'T_o');

subplot(2,3,5);
sigma(T_di_to_y1, 'b');
title('Singular Values: SoG');
legend('SoG');

subplot(2,3,6);
sigma(C_e, 'r', C_e_min, 'b');
title('Singular Values: Ce, Ce_{red}');
legend('Ce', 'Ce_{red}');

% Annotate the plots
% (Optional) Add annotations for critical frequency points if needed

% Compute norms for individual transfer functions
gamma_1 = norm(T_r_to_e1, 'inf');
gamma_2 = norm(T_r_to_u, 'inf');
gamma_3 = norm(T_r_to_e1_d, 'inf');

% Display individual performance levels
disp('Individual Performance Levels gamma_i:');
disp(['gamma_1: ', num2str(gamma_1)]);
disp(['gamma_2: ', num2str(gamma_2)]);
disp(['gamma_3: ', num2str(gamma_3)]);

% Verify the global performance level
norm_T_inf = norm(T, 'inf');
disp('Global Performance Level (norm of T):');
disp(norm_T_inf);

%%
% Load the OpenLoop_Test Simulink model

% Define the I/O points for linearization
% Break the loop at the input of the actuator (u_p) and output (y)
io_open(1) = linio('OpenLoop_Test/In', 1, 'input');
io_open(2) = linio('OpenLoop_Test/Gain', 1, 'output');

% Linearize the model
open_loop_tf = linearize('OpenLoop_Test', io_open);

% Plot the Bode plot to analyze gain and phase margins
figure;
[mag, phase, wout] = bode(open_loop_tf);
margin(open_loop_tf);
title('Bode Plot of the Open Loop Transfer Function');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB) / Phase (deg)');

% Compute gain and phase margins
[Gm, Pm, Wcg, Wcp] = margin(open_loop_tf);
disp(['Gain Margin (dB): ', num2str(20*log10(Gm))]);
disp(['Phase Margin (deg): ', num2str(Pm)]);
disp(['Gain Crossover Frequency (rad/s): ', num2str(Wcg)]);
disp(['Phase Crossover Frequency (rad/s): ', num2str(Wcp)]);

% Compute delay margin
DM = Pm / (180/pi) / Wcg;  % Convert PM from degrees to radians
DM_ms = DM * 1000;  % Convert from seconds to milliseconds
disp(['Delay Margin (ms): ', num2str(DM_ms)]);

% Annotate the gain at 300 rad/s
hold on;
gain_at_300_rad_s = bode(open_loop_tf, 300);
gain_at_300_db = 20*log10(gain_at_300_rad_s);
plot(300, gain_at_300_db, 'ro'); % mark the gain at 300 rad/s
text(300, gain_at_300_db, ['Gain at 300 rad/s: ', num2str(gain_at_300_db), ' dB']);
hold off;

%% Part #3C.3 Exercise 3
clc
close all
% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);
io = load('io_ClosedLoop_Test.mat');

% Linearize the model
T = linearize(model, io);

% Extract required transfer functions
So = T(1,1);   % T(1,1) = S_o
T_o = T(3,1);   % T(3,1) = T_o
Tr_udot_m = T(6,1); % T(6,1)
SoG = T(3,2);  % T(3,2) = SoG

% Time vector for simulation
t = linspace(0, 5, 1000);

% Step response plots
figure;
subplot(2, 2, 1);
step(So, t, 'b'); title('Step Response of So');
legend('So');

subplot(2, 2, 2);
step(T_o, t, 'b'); hold on; step(T_d, t, 'r'); title('Step Response of To and Td');
legend('To', 'Td');

subplot(2, 2, 3);
step(SoG, t, 'b'); title('Step Response of SoG');
legend('SoG');

subplot(2, 2, 4);
step(Tr_udot_m * (180/pi), t, 'b'); title('Step Response of Tr -> \omega_m');
legend('Tr -> \omega_m');
ylabel('Degrees/Second');
hold off;

% Use the stepinfo function to obtain the desired metrics
info_So = stepinfo(So);
info_SoG = stepinfo(SoG);
info_Td = stepinfo(T_d);
info_To = stepinfo(T_o);
info_Tr_udot_m = stepinfo(Tr_udot_m * (180/pi));

% Display the metrics in a table
metrics_table = table({'So', 'SoG', 'Td', 'To', 'Tr -> \omega_m'}', ...
    [info_So.RiseTime, info_SoG.RiseTime, info_Td.RiseTime, info_To.RiseTime, info_Tr_udot_m.RiseTime]', ...
    [info_So.SettlingTime, info_SoG.SettlingTime, info_Td.SettlingTime, info_To.SettlingTime, info_Tr_udot_m.SettlingTime]', ...
    [info_So.Peak, info_SoG.Peak, info_Td.Peak, info_To.Peak, info_Tr_udot_m.Peak]', ...
    'VariableNames', {'TransferFunction', 'RiseTime', 'SettlingTime', 'Peak'});

disp(metrics_table);




%% Part #3d - Feedback controller design (hinfstruct case)

% Controller design
C_E_red = tunableTF('C_e_red', 2, 2) * tf(1, [1 0]);

opts = hinfstructOptions('RandomStart', 20, 'UseParallel', true, 'TolGain', 1e-5);
[C_E_red, gamma_red, ~] = hinfstruct(P, C_E_red, opts);

C_E_red = tf(C_E_red);

C_I_red = minreal(tf(C_E_red * tf([1 0], 1)));

save('C_I_red.mat', 'C_I_red')

T_wz_new = tf(lft(P, C_E_red, 1, 1));

% Compute norms for individual transfer functions
gamma_1_new = norm(T_wz_new(1,:), 'inf');
gamma_2_new = norm(T_wz_new(2,:), 'inf');
gamma_3_new = norm(T_wz_new(3,:), 'inf');

% Display individual performance levels
disp('Individual Performance Levels gamma_i:');
disp(['gamma_1: ', num2str(gamma_1_new)]);
disp(['gamma_2: ', num2str(gamma_2_new)]);
disp(['gamma_3: ', num2str(gamma_3_new)]);

% Plot the singular values of T_wz and its individual components
figure;
hold on;
bode(C_i_min, 'b--');
bode(C_i_red, 'g--');
bode(C_I_red, 'magenta--')

legend('C_i_min (7 states hinfsys)', ...
       'C_i_red (2 states hinfsys)', ...
       'C_I_red (2 states hinfstruct))');

title('C_i stuff');
xlabel('Frequency (rad/s)');
ylabel('Gain stuff (Amplitude)');
hold off;

%%
% %% Part #3e - Feedforward controller design
% F_f  = tf(1, 1); % Unitary gain
% 
% % Open the Simulink model
% load_system('ClosedLoop_Red');
% 
% % Specify the input and output points for linearization
% % io(1) = linio('ClosedLoop_Red/r', 1, 'input');
% % io(2) = linio('ClosedLoop_Red/y1_out', 1, 'output');
% 
% % Linearize the model
% T_o = linearize('ClosedLoop_Red');   % Here the uncertainties are neglected
% T_o.InputName = {'r'};
% T_o.OutputName = {'y1'};
% 
% % S_o = 1/(1+G_cl_q_sc*C_E_red);
% % T_o = minreal(1 - S_o);
% F_f = minreal(T_o * inv(T_d));
% 
% 
% % Extract poles and zeros from the transfer function C0_e
% original_poles = pole(F_f);
% original_zeros = zero(F_f);
% 
% % Identify poles and zeros to keep based on the thresholds
% poles_to_keep = original_poles([false false true true true true true false]);
% zeros_to_keep = original_zeros([false true true true true true]);
% 
% % Create the new transfer function with the kept poles and zeros
% F_f_min = zpk(zeros_to_keep, poles_to_keep, db2mag(12.345));
% 
% % Further reduce 
% R = reducespec(F_f_min,"balanced");
% F_f_red = getrom(R,Order=2);

%%
clear
load('G_a.mat'); load('G_m.mat'); load('C_q.mat'); load('C_sc.mat'); load('W1.mat'); load('W2.mat'); load('W3.mat');  

% Initialize feedback controller Ci with the one obtained from Question 3D.1
Ci = load('C_I_red.mat');  % Assume Ci is saved in this file

% Initialize feedforward controller Ff to unity
F_f  = tf(1, 1);

% Open the ClosedLoop_Test Simulink model
model = 'ClosedLoop_Test';
open_system(model);
