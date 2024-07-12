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
save G_m G_a G_am;

clearvars -except G_a G_am G_m

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

save C_q G_cl_q_unsc;

clearvars -except G_a G_am G_m G_cl_q_unsc C_q

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

save C_sc G;

clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G

%% Scaling Gain Design - Disturbance Rejection (5%)



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

save C_i_pm

clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm T

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

clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm C_i_st
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

clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm C_i_st
%% Part #3a – Weighting filters (2.5%)

% Low-pass filter parameters W_1
% Penalises low frequency deviations tracking errors.
dcgain_W_1 = 1e3;           % high gain at low frequencies
hfgain_W_1 = 1e-3;        % zero gain at high frequencies
mag_W_1 = db2mag(-3.01);  % -3.01 dB at 4 rad/s
freq_W_1 = 4.0;           % [rad/s]

% High-pass filter parameters W_2, assume M_1 = A_2 , A_1 = M_2
% Penalises high frequency inputs to the system.
dcgain_W_2 = hfgain_W_1;  % zero gain at low frequencies
hfgain_W_2 = dcgain_W_1;  % high gain at high frequencies
mag_W_2 = db2mag(-15);    % -15 dB at -3.01dB bandwidth frequency
freq_W_2 = 151;           % freq where magnitude of actuator dynamics equals -3.01dB [rad/s]

% Construct W_1 and W_2
W_1 = makeweight(dcgain_W_1,[freq_W_1,mag_W_1],hfgain_W_1);
W_2 = makeweight(dcgain_W_2,[freq_W_2,mag_W_2],hfgain_W_2);

% Compute M_1, A_1, and omega_1
M_1 = 1/hfgain_W_1;                             % gain at high frequencies
A_1 = 1/dcgain_W_1;                             % gain at low frequencies
omega_1 = freq_W_1 / sqrt((1 / mag_W_1^2) - 1); % natural frequency

% Compute M_2, A_2, and omega_2
M_2 = 1/hfgain_W_2;                             % gain at high frequencies
A_2 = 1/dcgain_W_2;                             % gain at low frequencies
omega_2 = freq_W_2 * sqrt((1 / mag_W_2^2) - 1); % natural frequency

% Define the transfer functions for W_1 and W_2 for verification
W_1_tf = tf([1 / M_1 omega_1], [1 omega_1 * A_1]);
W_2_tf = tf([1 omega_2/A_2], [M_2 omega_2]);

[~, phase_margin, ~, ~] = margin(W_1_tf);

% Save the filters for later use
save('W1.mat', 'W_1');
save('W2.mat', 'W_2');

clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm C_i_st W_1_tf W_2_tf W_1 W_2

%%
close all

figure;
bode(W_1_tf)
hold on;
bode(W_2_tf)
legend('W1', 'W2');

%Plot the filters' inverse gain
figure;
sigma(inv(W_1_tf), 'r', inv(W_2_tf), 'b');
grid on;
legend('1/W1', '1/W2');
title('Inverse Gain of Weighting Filters');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');

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

omega_d = x_opt(1)
zeta_d = x_opt(2)

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
save T_d;

% Display the transfer function in zero-pole-gain form
%T_d_zpk = zpk(T_d);
%disp('Reference model transfer function T_d:');
%disp(T_d_zpk);

clearvars -except G_a G_am G_m G_cl_q_unsc C_q C_sc G C_i_pm C_i_st W_1_tf W_2_tf W_1 W_2 T_d


%% Part #3c - Feedback controller design (hinfsyn case)
clc 
% Low-pass filter parameters W_3
% Penalises difference between reference and state
dcgain_W_3 = 1e3;         % high gain at low frequencies
hfgain_W_3 = 1e-3;        % zero gain at high frequencies
mag_W_3 = db2mag(-3.01); % -3.01 dB at 4 rad/s
freq_W_3 = 4;         % [rad/s]

% Compute M_3, A_3, and omega_3
M_3 = 1/hfgain_W_3;                             % gain at high frequencies
A_3 = 1/dcgain_W_3;                             % gain at low frequencies
omega_3 = freq_W_3 / sqrt((1 / mag_W_3^2) - 1); % natural frequency

W_3_tf = tf([1 / M_3 omega_3], [1 omega_3 * A_3]);
 % Set W3 equal to W1 for relaxed model following constraint

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

% Linearize the model
P = linearize('Design', io);   % Here the uncertainties are neglected
P.InputName = {'w', 'u'};
P.OutputName = {'w1', 'w2', 'w3', 'v'};


%P = [W_1 -W_1*G; 0 W_2; W_3*T_d -W_3*G; 1 -G];
%P.InputName = {'w', 'u'};
%P.OutputName = {'w1', 'w2', 'w3', 'v'};

save('P.mat', 'P');

%% 3C.1 Exercise 3

% Define the number of measurements and controls
nmeas = 1; % Number of measurements (outputs to the controller)
nctrl = 1; % Number of controls (inputs from the controller)

% Set optimization options for hinfsyn
opts = hinfsynOptions('RelTol', 1e-6);

% Design the H-infinity controller using hinfsyn
[C_e, T_wz, gamma] = hinfsyn(P, nmeas, nctrl, opts);

save('C_e.mat', 'C_e')

% Compute the closed-loop transfer function with weights
S_o = 1/(1+G*C_e);
T_o = 1 - S_o;
%T_wz = [W1 * S_o; W2 * C_e * S_o; W3 * (T_d - T_o)];

%%
% Plot the singular values of T_wz and its individual components
figure;
sigma(T_wz);
hold on;
sigma(T_wz(1,:), 'r--');
sigma(T_wz(2,:), 'g--');
sigma(T_wz(3,:), 'b--');
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


%% 3C.2 Controller order reduction (5%)
% Load the initial controller C0_e (example given, replace with actual data)
% Assuming C0_e is a transfer function object
%load('C0_e.mat'); % Replace with actual loading mechanism if different
close all; clc
%pzmap(minreal(C0_e))
F_f = 1;
C0_e = C_e;
zpk(C0_e)
%pole(C0_e)
zero(C0_e)
zpkdata(C0_e,'v')

% Define thresholds
high_threshold = 1e4;  % Example threshold for very high values
low_threshold = 0.0030;  % Example threshold for very low values


% Original poles
poles = [-1.4256e+06 + 0i, -88.37 + 93.301i, -88.37 - 93.301i, -22.93 + 19.053i, -22.93 - 19.053i, ...
         -12.948 + 12.926i, -12.948 - 12.926i, -0.0021061 + 0i, -0.0028285 + 0i];
zeros = [-28118 + 0i, -91.366 +     95.336i, -91.366 -     95.336i, -14.287 +     14.574i, -14.287 -     14.574i, -12.98 +     12.949i, -12.98 -     12.949i, -0.0028285 +          0i];
% Poles to remove
poles_to_remove = [-1.4256e+06 + 0i, -0.0028285 + 0i];
zeros_to_remove = [-28118 + 0i, -0.0028285 +          0i];
% New set of poles
new_poles = setdiff(poles, poles_to_remove); 
new_zeros = setdiff(zeros, zeros_to_remove);

% Gain of the transfer function (modify according to your system)
gain = mag2db(-1.4256e+06/-28118)

-1.4256e+06/-28118
% Create the new transfer function
C_e_min = zpk(new_zeros, new_poles, gain)


% Remove extreme poles
%C_e_min = remove_extreme_poles_zeros(C0_e, high_threshold, low_threshold);

pole(C_e_min) 
zero(C_e_min)
figure;
bode(C0_e)
hold on;
bode(C_e_min)
%%
close all
R = reducespec(C0_e,"balanced");
figure;
view(R)

rsys = getrom(R,Order=5);
pole(rsys)
zero(rsys)
figure;
bode(C0_e,rsys,'r--')
legend("Original","Order 7")
%%
figure;
iopzmap(C_e)
hold on;
iopzmap(rsys)

figure;
bode(C0_e,rsys,'r--')
legend("Original","Order 6")
% %size(C0_e)
% zero(C0_e)
% %size(minreal(C0_e))
% zero(minreal(C0_e))
% 
% figure;
% bode(C0_e)
% hold on;
% bode(minreal(C0_e))

%%
% Display full order controller details
disp('Full Order Controller C0_e:')
zpk(C0_e)

% Obtain ZPK data
[z, p, k] = zpkdata(C0_e, 'v');

% Reform the minimal form controller C_e_min
% Retain necessary zeros and poles, adjust gain
z_min = z(1:6); % Retain first 6 zeros (example indices)
p_min = p(1:7); % Retain first 7 poles (example indices)
k_min = k * (prod(p_min) / prod(z_min)); % Adjust gain

C_e_min = zpk(z_min, p_min, k_min);
disp('Minimal Form Controller C_e_min:')
zpk(C_e_min)

% Obtain the integral controller C_i_min
C_i_min = C_e_min / tf('s');

C_i_min = minreal(tf(C_e_min * tf([1 0], 1)));

disp('Integral Controller C_i_min:')
zpk(C_i_min)

% Perform model order reduction
C_i_red = balred(C_i_min, 2); % Example order reduction to 2nd order

disp('Reduced Order Integral Controller C_i_red:')
zpk(C_i_red)

% Plot Bode plots
figure;
bode(C0_e, 'r', C_e_min, 'b--');
legend('C0_e', 'C_e_min');
title('Bode Plot of Full and Simplified Controllers');

figure;
bode(C_i_min, 'r', C_i_red, 'b--');
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


%% Part #3d - Feedback controller design (hinfstruct case)

% Controller design
C_E_red = tunableTF('C_e_red', 2, 2) * tf(1, [1 0]);

opts = hinfstructOptions('RandomStart', 20, 'UseParallel', true, 'TolGain', 1e-5);
[C_E_red, gamma_red, ~] = hinfstruct(P, C_E_red, opts);

C_I_red = minreal(tf(C_E_red * tf([1 0], 1)));

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
%bode(C_i_red, 'g--');
bode(C_I_red, 'magenta--')

legend('(T_{wz})', ...
       '\sigma(T_{wz1})', ...
       '\sigma(T_{wz2})', ...
       '\sigma(T_{wz3})');

title('C_i stuff');
xlabel('Frequency (rad/s)');
ylabel('Gain stuff (Amplitude)');
hold off;








function new_tf = remove_extreme_poles_zeros(C0_e, high_threshold, low_threshold)
    % Extract poles and zeros from the transfer function C0_e
    original_poles = pole(C0_e);
    original_zeros = zero(C0_e);
    
    % Identify poles and zeros to keep based on the thresholds
    poles_to_keep = original_poles(abs(original_poles) <= high_threshold & abs(original_poles) >= low_threshold);
    zeros_to_keep = original_zeros(abs(original_zeros) <= high_threshold & abs(original_zeros) >= low_threshold);
    
    % Create the new transfer function with the kept poles and zeros
    %gain = dcgain(C0_e);  % Extract gain from C0_e
    gain = 1000;
    new_tf = zpk(zeros_to_keep, poles_to_keep, gain);
end
