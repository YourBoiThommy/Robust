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

% Scaling Gain Design (5%)
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


% Integral Gain Design (10%) sisotool

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

% Integral Gain Design (10%) settling time C_i_st

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
% Ploting
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
M = 1.9;                  % high gain at low frequencies while also ensuring PM above 30 degrees
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

%% Part 3C.1 - Feedback controller design (hinfsyn case)
clc 
clear io
% Low-pass filter parameters W_3
W_3 = W_1; % Set W3 equal to W1 for relaxed model following constraint

% Open model
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
P = linearize('Design', io);  
P.InputName = {'w', 'u'};
P.OutputName = {'z1', 'z2', 'z3', 'v'};

save('P.mat', 'P');
zpk(P)
zpk(G)
zpk(W_1)
zpk(W_2)


%%
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
freq_range = {0.001, 10000}; % Frequency range from 1 to 300 rad/s

% Plot the singular values of T_wz and its individual components
figure;
sigma(T_wz, freq_range);
hold on;
sigma(T_wz(1,:), freq_range, 'r--');
sigma(T_wz(2,:), freq_range, 'g--');
sigma(T_wz(3,:), freq_range, 'b--');
legend('$\sigma(T_{wz})$', '$\sigma(T_{wz1})$', '$\sigma(T_{wz2})$', '$\sigma(T_{wz3})$', 'interpreter', 'latex');
title('Singular Value Plot');
xlabel('Frequency');
ylabel('Singular Value');
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
mag_W_3 = db2mag(17);   % at -17.1 dB gamma_2 reaches exactly 1
W_3 = tf(makeweight(1/A, [freq_W_1, mag_W_3], 1/M));

save('W3.mat', 'W_3')

load_system('Design');
io = load('io_Design.mat').io;

% Linearize the model
P = linearize('Design', io); 
P.InputName = {'w', 'u'};
P.OutputName = {'z1', 'z2', 'z3', 'v'};

zpk(W_1)
zpk(W_2)
zpk(W_3)

% Design the H-infinity controller using hinfsyn
[C_e, T_wz, gamma] = hinfsyn(P, nmeas, nctrl, opts);

% Compute the closed-loop transfer function with weights
S_o = 1/(1+G*C_e);
T_o = 1 - S_o;
freq_range = {0.001, 10000}; % Frequency range from 1 to 300 rad/s

% Plot the singular values of T_wz and its individual components
figure;
sigma(T_wz, freq_range);
hold on;
sigma(T_wz(1,:), freq_range, 'r--');
sigma(T_wz(2,:), freq_range, 'g--');
sigma(T_wz(3,:), freq_range, 'b--');
legend('$\sigma(T_{wz})$', '$\sigma(T_{wz1})$', '$\sigma(T_{wz2})$', '$\sigma(T_{wz3})$', 'interpreter', 'latex');
title('Singular Value Plot');
xlabel('Frequency');
ylabel('Singular Value');
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
poles = pole(C0_e);
zeros = zero(C0_e);

% Identify poles and zeros to keep
poles_min = poles([false true true true true true true true false]);
zeros_min = zeros([false true true true true true true false]);

k_new = k * abs(zeros(1))/abs(poles(1));

% Create the new transfer function with the kept poles and zeros
C_e_min = zpk(zeros_min, poles_min, k_new);
[z, p, k] = zpkdata(C_e_min,'v');

poles_min = poles([false true true true true true true false false]);
C_i_min = zpk(zeros_min, poles_min, k);

R = reducespec(C_i_min,"balanced");
figure;
view(R)
C_i_red = getrom(R,Order=2);

disp('Minimum Integral Controller C_i_min: ')
zpk(C_i_min)

disp('Reduced Integral Controller C_i_red: ')
zpk(C_i_red)

figure;
freq_range = {0.01, 100000};
bode(C0_e, 'r', C_e_min, 'b--', freq_range);
legend("Original", "Order 7");
grid on;
legend('C0_e', 'C_e_min');
title('Bode Plot of the Full and Simplified Controller');

figure;
freq_range = {0.01, 100000};
bode(C_i_min, 'r', C_i_red, 'b--', freq_range);
grid on;
legend('C\_i\_min', 'C\_i\_red');
title('Bode Plot of the Integral and Reduced Controller');

freq_range = logspace(-1, 4, 5000); % Frequency range from 0.1 to 1000 rad/s

% Compute the Bode plots for C_i_min and C_i_red
[~, phase_C_i_min] = bode(C_i_min, freq_range);
[~, phase_C_i_red] = bode(C_i_red, freq_range);

% Convert the phase data to a more usable format
phase_C_i_min = squeeze(phase_C_i_min);
phase_C_i_red = squeeze(phase_C_i_red);

% Plot the phase responses
figure;
semilogx(freq_range, phase_C_i_min, 'g', 'LineWidth', 1.5);
hold on;
semilogx(freq_range, phase_C_i_red, 'm--', 'LineWidth', 1.5);
legend('C\_i\_min', 'C\_i\_red', 'Location', 'Best');
title('Phase Response of C_i_min and C_i_red');
xlabel('Frequency (rad/s)');
ylabel('Phase (deg)');
grid on;

% Calculate the phase difference between the two controllers
phase_diff = phase_C_i_red - phase_C_i_min;

% Plot the phase difference
figure;
semilogx(freq_range, phase_diff, 'b', 'LineWidth', 1.5);
title('Phase Difference between C_i_min and C_i_red');
xlabel('Frequency (rad/s)');
ylabel('Phase Difference (deg)');
grid on;

% Plot pole-zero maps
figure;
pzmap(C_i_min, 'b', C_i_red, 'r' );
legend('C\_i\_min', 'C\_i\_red');
title('Pole-Zero Map of C\_i\_min and C\_i\_red');

% Save controllers if needed
save('C_e_min.mat', 'C_e_min');
save('C_i_red.mat', 'C_i_red');


%% Part #3C.3 - Controller analysis & simulation (5%)
close all;

% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);

C_i = C_i_red;
C_e_red = C_i_red * 1/(tf('s'));

% Define the I/O points for linearization
io(1) = linio([model '/r'], 1, 'input');   % Input r
io(2) = linio([model '/d_i'], 1, 'input'); % Input d_i
io(3) = linio([model '/Sum3'], 1, 'output'); % Output e1
io(4) = linio([model '/C_i'], 1, 'output');  % Output u
io(5) = linio([model '/Demux1'], 1, 'output'); % Output y1
io(6) = linio([model '/Sum1'], 1, 'output'); % Output e1_d
io(7) = linio([model '/Sum2'], 1, 'output'); % Output u_p
io(8) = linio([model '/Demux'], 2, 'output'); % Output udot_m

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
sigma(W1_inv, 'r', T_r_to_e1, 'b', T_di_to_up, 'b');
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
sigma(-T_di_to_u, 'b', T_r_to_y1, 'b');
title('Singular Values: T_i, T_o');
legend('T_i', 'T_o');

subplot(2,3,5);
sigma(T_di_to_y1, 'b');
title('Singular Values: SoG');
legend('SoG');

subplot(2,3,6);
sigma(C_e, 'r', C_e_red, 'b');
title('Singular Values: C\_e, C\_e\_red');
legend('C\_e', 'C\_e\_red');


% Break the loop at the input of the actuator (u_p) and output (y)
io_open(1) = linio('OpenLoop_Test/In', 1, 'input');
io_open(2) = linio('OpenLoop_Test/Gain', 1, 'output');
open_loop_tf = linearize('OpenLoop_Test', io_open);

figure;
h = bodeplot(open_loop_tf);
hold on;
margin(open_loop_tf);
% Compute gain and phase margins
[Gm, Pm, Wcg, Wcp] = margin(open_loop_tf);
disp(['Gain Margin (dB): ', num2str(20*log10(Gm))]);
disp(['Phase Margin (deg): ', num2str(Pm)]);
disp(['Phase Crossover Frequency (rad/s): ', num2str(Wcg)]);
disp(['Gain Crossover Frequency (rad/s): ', num2str(Wcp)]);

% Compute delay margin
DM = Pm / (180/pi) / Wcp;  % Convert PM from degrees to radians
DM_ms = DM * 1000;  % Convert from seconds to milliseconds
disp(['Delay Margin (ms): ', num2str(DM_ms)]);

% Annotate the gain at 300 rad/s
[mag, ~, wout] = bode(open_loop_tf, 300); % get magnitude at 300 rad/s
gain_at_300_db = round(20*log10(mag), 2);

% Get the axes handle for the magnitude plot
ax = getaxes(h);
mag_ax = ax(1);

% Plot the gain point on the magnitude subplot
axes(mag_ax);
plot(300, gain_at_300_db, 'ro'); % mark the gain at 300 rad/s
text(300, gain_at_300_db, ['Gain at 300 rad/s: ', num2str(gain_at_300_db), ' dB'], 'VerticalAlignment', 'bottom');
hold off;

%% Part #3C.3 Exercise 3
clc
close all
% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);
io = load('io_ClosedLoop_Test.mat').io;

% Linearize the model
T = linearize(model, io);

% Extract required transfer functions
So = T(1,1);   % T(1,1) = S_o
T_o = T(3,1);   % T(3,1) = T_o
Tr_udot_m = T(6,1); % T(6,1)
SoG = T(3,2);  % T(3,2) = SoG

% Time vector for simulation
t = linspace(0, 1, 10000);

% Step response plots
figure;
subplot(2, 2, 1);
step(So, t, 'b'); title('Step Response of So');
legend('So');

subplot(2, 2, 2);
step(T_o, 'b'); hold on; step(T_d, 'r'); title('Step Response of To and Td');
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
info_So = stepinfo(So,'SettlingTimeThreshold', 5/100);
info_SoG = stepinfo(SoG,'SettlingTimeThreshold', 5/100);
info_Td = stepinfo(T_d,'SettlingTimeThreshold', 5/100);
info_To = stepinfo(T_o,'SettlingTimeThreshold', 5/100);
info_Tr_udot_m = stepinfo(Tr_udot_m* (180/pi),'SettlingTimeThreshold', 5/100);

% Display the metrics in a table
metrics_table = table({'So', 'SoG', 'Td', 'To', 'Tr -> \omega_m'}', ...
    [info_So.TransientTime, info_SoG.TransientTime, info_Td.TransientTime, info_To.TransientTime, info_Tr_udot_m.TransientTime]', ...
    [info_So.SettlingTime, info_SoG.SettlingTime, info_Td.SettlingTime, info_To.SettlingTime, info_Tr_udot_m.SettlingTime]', ...
    [info_So.Peak, info_SoG.Peak, info_Td.Peak, info_To.Peak, info_Tr_udot_m.Peak]', ...
    'VariableNames', {'TransferFunction', 'TransientTime', 'SettlingTime', 'Peak'});

disp(metrics_table);


zpk(T_d)

%% Part #3D.1 - Feedback controller design (hinfstruct case)
clc
close all

s = tf([1,0],1);
C_e_red_i = tunableTF('C_e_red_s', 2, 2) / s;

options = hinfstructOptions('RandomStart', 25, 'UseParallel', true, 'TolGain', 1e-8);
[C_e_red_s, gamma_s, info] = hinfstruct(P, C_e_red_i, options);
C_i_red_s = minreal(tf(C_e_red_s)*s);

T_wz_struct = tf(lft(P, C_e_red_s, 1, 1));

T_wz_s_1 = T_wz_struct(1);
T_wz_s_2 = T_wz_struct(2);
T_wz_s_3 = T_wz_struct(3);

% Compute norms for individual transfer functions
gamma_1_struct = norm(T_wz_s_1, 'inf');
gamma_2_struct = norm(T_wz_s_2, 'inf');
gamma_3_struct = norm(T_wz_s_3, 'inf');

% Display individual performance levels
disp('Individual Performance Levels with hinfstruct:');
disp(['gamma_1: ', num2str(gamma_1_struct)]);
disp(['gamma_2: ', num2str(gamma_2_struct)]);
disp(['gamma_3: ', num2str(gamma_3_struct)]);

%% Part #3D.1 - Plotting

figure;
hold on;
bode(C_i_min, 'b--');
bode(C_i_red, 'g--');
bode(C_i_red_s, 'magenta--')

legend('C_{i_{min}}', ...
       'C_{i_{red}}', ...
       'C_{i_{red}}^*');

title('hinfsys and hinfstruct comparison');
hold off;

% Plot singular values
figure;
hold on;
sigma(T_wz_s_1);
sigma(T_wz_s_2);
sigma(T_wz_s_3);
legend('T_wz_1', 'T_wz_2', 'T_wz_3');
title('T_{wz}^* Singular values')
hold off;

%% Part #3D.2
close all

C_i = C_i_red_s;

% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);
io = load('io_ClosedLoop_Test.mat').io;

% Linearize the model
T = linearize(model, io);

% Extract required transfer functions
So_s = T(1,1);   % T(1,1) = S_o
CeSo_s = T(2,1);    % T(2,1) = CeS_o
To_s = T(3,1);   % T(3,1) = T_o
T_m_s = T(4,1); % T(4,1) = T_m
T_rum_s = T(6,1); % T(6,1)
T_diu_s = T(2,2);   % T(2,2) = -T_i
SoG_s = T(3,2);  % T(3,2) = SoG
T_diup_s = T(5,2);  % T(5,2) = S_i

% Define the weighting filter inverses for comparison
W1_inv = inv(W_1);
W2_inv = inv(W_2);
W3_inv = inv(W_3);

% Plot singular values
figure;

subplot(2,3,1);
sigma(W1_inv, 'r', T_r_to_e1, 'b', So_s, 'magenta');
title('Singular Values: S_o');
legend('W1^{-1}', 'hinfsys', 'hinfstruct');

subplot(2,3,2);
sigma(W2_inv, 'r', T_r_to_u, 'b', CeSo_s, 'magenta');
title('Singular Values: C_eS_o');
legend('W2^{-1}', 'hinfsys', 'hinfstruct');

subplot(2,3,3);
sigma(W3_inv, 'r', T_r_to_e1_d, 'b', T_m_s, 'magenta');
title('Singular Values: T_m');
legend('W3^{-1}', 'hinfsys', 'hinfstruct')

subplot(2,3,4);
sigma(T_r_to_y1, 'b', To_s, 'magenta');
title('Singular Values: T_o');
legend('hinfsys', 'hinfstruct');

subplot(2,3,5);
sigma(T_di_to_y1, 'b', SoG_s, 'magenta');
title('Singular Values: SoG');
legend('hinfsys', 'hinfstruct')

subplot(2,3,6);
sigma(C_e, 'r', C_e_min, 'b', C_e_red_s, 'magenta');
title('Singular Values: Ce, Ce_{red}');
legend('Ce', 'hinfsys', 'hinfstruct');

% Load the OpenLoop_Test Simulink model

% Linearize the model
open_loop_tf_str = linearize('OpenLoop_Test', io_open);

% Plot the Bode plot to analyze gain and phase margins
figure;
hold on;
bode(open_loop_tf,'b');
bode(open_loop_tf_str, 'magenta');
margin(open_loop_tf)
margin(open_loop_tf_str);
title('Bode Plot of the Open Loop Transfer Function');
legend('hinfsys', 'hinfstruct')

% Compute gain and phase margins
[Gm_str, Pm_str, Wcg_str, Wcp_str] = margin(open_loop_tf_str);
disp(['Gain Margin (dB): ', num2str(20*log10(Gm_str))]);
disp(['Phase Margin (deg): ', num2str(Pm_str)]);
disp(['Gain Crossover Frequency (rad/s): ', num2str(Wcg_str)]);
disp(['Phase Crossover Frequency (rad/s): ', num2str(Wcp_str)]);

% Compute delay margin
DM_str = Pm_str / (180/pi) / Wcg_str;  % Convert PM from degrees to radians
DM_ms_str = DM_str * 1000;  % Convert from seconds to milliseconds
disp(['Delay Margin (ms): ', num2str(DM_ms_str)]);

% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);
io = load('io_ClosedLoop_Test.mat').io;

% Linearize the model
T = linearize(model, io);
% Extract required transfer functions
So = T(1,1);   % T(1,1) = S_o
T_o = T(3,1);   % T(3,1) = T_o
Tr_udot_m = T(6,1); % T(6,1)
SoG = T(3,2);  % T(3,2) = SoG

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
% Step response plots
figure;
subplot(2, 2, 1);
step(So, T(1,1), 'b'); title('Step Response of So');
legend('So');

subplot(2, 2, 2);
step(T_o, T(3,1), 'b'); hold on; step(T_d, t, 'r'); title('Step Response of To and Td');
legend('To', 'Td');

subplot(2, 2, 3);
step(SoG, T(3,2), 'b'); title('Step Response of SoG');
legend('SoG');

subplot(2, 2, 4);
step(Tr_udot_m, T(6,1) * (180/pi), 'b'); title('Step Response of Tr -> \omega_m');
legend('Tr -> \omega_m');
ylabel('Degrees/Second');

%% Part #3e - Feedforward controller design

F_f_init = T_d * inv(To_s);
[orig_zeros,orig_poles,orig_gain] = zpkdata(F_f_init,'v');

% Identify poles and zeros to keep
mod_zeros = orig_zeros(abs(orig_zeros) < 100 & real(orig_zeros) < 0);
mod_poles = orig_poles(real(orig_poles) < 0);

% Create the new transfer function with the kept poles and zeros
F_f_lf = minreal(zpk(mod_zeros, mod_poles, 1));
F_f_lf = F_f_lf / dcgain(F_f_lf);

% Further reduce 
R = reducespec(F_f_lf,"balanced");
F_f = getrom(R,Order=3);

figure;
hold on;
bode(F_f);
bode(F_f_lf);
bode(F_f_init);
legend('Reduced', 'Low Frequency', 'Initial');

% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);
io = load('io_ClosedLoop_Test.mat').io;

% Linearize the model
T = linearize(model, io);
T_m_ff = T(4,1);
T_o_ff = T(3,1);
T_rum_ff = T(6,1);

figure;
subplot(2, 2, 1);
sigma(W3_inv, 'r', T_r_to_e1_d, 'b', T_m_s, 'magenta', T_m_ff, 'g');
title('Singular Values: T_m');
legend('W3^{-1}', 'hinfsys', 'hinfstruct', 'feedforward');

subplot(2, 2, 2);
sigma(C_i_red, 'r', C_i_red_s, 'b', F_f, 'magenta');
title('Singular Values: T_o');
legend('hinfsys', 'hinfstruct', 'feedforward');

subplot(2, 2, 3);
step(T_d, 'r', T_r_to_y1, 'b', To_s, 'magenta', T_o_ff, 'g');
title('Step Responses: T_o');
legend('Reference', 'hinfsys', 'hinfstruct', 'feedforward');

subplot(2, 2, 4);
step(T_r_to_udot_m*(180/pi), 'b', T_rum*(180/pi), 'magenta', T_rum_ff*(180/pi), 'g');
title('Step Responses: T_{r_{\dot{u}_m}}');
legend('hinfsys', 'hinfstruct', 'feedforward');

%% Cell Part %4
clear
% Load all relevant data
load('G_a.mat'); load('G_m.mat'); load('C_q.mat'); load('C_sc.mat'); load('W1.mat'); load('W2.mat'); load('W3.mat');  
C_i = load('C_I_red.mat').C_I_red; 
T_d = load('T_d.mat').T_d;

% Initialize feedforward controller Ff to unity
F_f  = tf(1, 1);

% Right now the maximum achievable attenuation is -17dB but -30dB should be
% attainable, first let's change W_3
mag_W_3 = db2mag(31); 

W_3 = tf(makeweight(1/0.001, [4, mag_W_3], 1/1.9));

% Create system data with slTuner interface
TunedBlocks = {'ClosedLoop_Test/C_i/LTI System'};
AnalysisPoints = {'ClosedLoop_Test/r/1'; ...
                  'ClosedLoop_Test/C_i/1'; ...
                  'ClosedLoop_Test/Demux1/1'; ...
                  'ClosedLoop_Test/Reference Model/1'; ...
                  'ClosedLoop_Test/Sum1/1'; ...
                  'ClosedLoop_Test/Sum3/1'};
% Specify the custom options
Options = slTunerOptions('AreParamsTunable',false);
% Create the slTuner object
CL0 = slTuner('ClosedLoop_Test',TunedBlocks,AnalysisPoints,Options);

ClosedLoop_Test_C_i_LTI_System = tunableTF('ClosedLoop_Test_C_i_LTI_System',2,2);
ClosedLoop_Test_C_i_LTI_System.Numerator.Value = [26.8830773604334 783.409142396318 12264.8389344962];
ClosedLoop_Test_C_i_LTI_System.Denominator.Value = [1 62.9461536531052 1590.02149251343];
setBlockParam(CL0,'ClosedLoop_Test/C_i/LTI System',ClosedLoop_Test_C_i_LTI_System);

% Create tuning goal to limit gain of an I/O transfer function
% Inputs and outputs
Inputs = {'ClosedLoop_Test/r/1[r]'};
Outputs = {'ClosedLoop_Test/Sum3/1[e1]'};
% Tuning goal specifications
MaxGain = 1/W_1; % Maximum gain as a function of frequency
% Create tuning goal for gain
So = TuningGoal.Gain(Inputs,Outputs,MaxGain);
So.Name = 'So'; % Tuning goal name

% Create tuning goal to limit gain of an I/O transfer function
% Inputs and outputs
Inputs = {'ClosedLoop_Test/r/1[r]'};
Outputs = {'ClosedLoop_Test/C_i/1[u]'};
% Tuning goal specifications
MaxGain = 1/W_2; % Maximum gain as a function of frequency
% Create tuning goal for gain
CeSo = TuningGoal.Gain(Inputs,Outputs,MaxGain);
CeSo.Name = 'CeSo'; % Tuning goal name

% Create tuning goal to limit gain of an I/O transfer function
% Inputs and outputs
Inputs = {'ClosedLoop_Test/r/1[r]'};
Outputs = {'ClosedLoop_Test/Sum1/1[e1_d]'};
% Tuning goal specifications
MaxGain = 1/W_3; % Maximum gain as a function of frequency
% Create tuning goal for gain
Tm = TuningGoal.Gain(Inputs,Outputs,MaxGain);
Tm.Name = 'Tm'; % Tuning goal name

% Create option set for systune command
Options = systuneOptions();
Options.RandomStart = 30; % Number of randomized starts
Options.UseParallel = true; % Parallel processing flag

% Set soft and hard goals
SoftGoals = [];
HardGoals = [ So ; ...
              CeSo ; ...
              Tm ];

% Tune the parameters with soft and hard goals
[CL1,fSoft,gHard,Info] = systune(CL0,SoftGoals,HardGoals,Options);

%% View tuning results
% viewSpec([SoftGoals;HardGoals],CL1);

getBlockValue(CL1,'C')

%%
Numerator = [26.780513435714976,7.264664166702048e+02,1.233894195762681e+04];
Denominator = [1,53.275177212832276,1.367670325448250e+03];
close all;
zpk(C_i)
C_sharp_I_red = tf(Numerator, Denominator);
C_i = C_sharp_I_red; 
zpk(C_sharp_I_red)
figure;
hold on;
pzmap(load('C_I_red.mat').C_I_red)
pzmap(C_sharp_I_red)
legend('$C_{i, red}^*$ (hinfstruct)', 'C$_{i, red}^\#$ (systune)', 'interpreter','latex')
hold off;

% Load the Simulink model
model = 'ClosedLoop_Test';
load_system(model);
io = load('io_ClosedLoop_Test.mat').io;

% Linearize the model
T = linearize(model, io);

% Extract required transfer functions
So = T(1,1);   % T(1,1) = S_o
T_o = T(3,1);   % T(3,1) = T_o
Tr_udot_m = T(6,1); % T(6,1)
SoG = T(3,2);  % T(3,2) = SoG

% Time vector for simulation
t = linspace(0, 1, 1000);

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
info_So = stepinfo(So,'SettlingTimeThreshold', 5/100);
info_SoG = stepinfo(SoG,'SettlingTimeThreshold', 5/100);
info_Td = stepinfo(T_d,'SettlingTimeThreshold', 5/100);
info_To = stepinfo(T_o,'SettlingTimeThreshold', 5/100);
info_Tr_udot_m = stepinfo(Tr_udot_m* (180/pi),'SettlingTimeThreshold', 5/100);

% Display the metrics in a table
metrics_table = table({'So', 'SoG', 'Td', 'To', 'Tr -> \omega_m'}', ...
    [info_So.RiseTime, info_SoG.RiseTime, info_Td.RiseTime, info_To.RiseTime, info_Tr_udot_m.RiseTime]', ...
    [info_So.SettlingTime, info_SoG.SettlingTime, info_Td.SettlingTime, info_To.SettlingTime, info_Tr_udot_m.SettlingTime]', ...
    [info_So.Peak, info_SoG.Peak, info_Td.Peak, info_To.Peak, info_Tr_udot_m.Peak]', ...
    'VariableNames', {'TransferFunction', 'RiseTime', 'SettlingTime', 'Peak'});

disp(metrics_table);

% Display individual performance levels
%disp('Individual Performance Levels gamma_i:');
%disp(['gamma_1: ', num2str(gHard(1))]);
%disp(['gamma_2: ', num2str(gHard(2))]);
%disp(['gamma_3: ', num2str(gHard(3))]);