%% Bearing Reaction Forces
dy_faceWidthPulley = 22.225; % mm
dy_supportBox = 100;
theta = 68.3;
F_pulley = [0, -130.7, 0]; % N
%fprintf("Pulley Force: [%di, %.2fj, %dk]\n", F_pulley(1), F_pulley(2), F_pulley(3));
F_mag_blade = 42.8;
%fprintf("Blade Magnitude Force: %.2f N\n", F_mag_blade);
F_blade = [F_mag_blade*cosd(theta), 0, F_mag_blade*sind(theta)];
%fprintf("Blade Vector Force: [%.2fi, %dj, %.2fk]\n", F_blade(1), F_blade(2), F_blade(3));
F_mp = [0, 10, 0];
F_mn = [0, -10, 0];
dy_halfFaceWidthPulley = dy_faceWidthPulley / 2;
dy_supportBox_halfFaceWidthPulley = 50;
dy_widthBearing = 13;
dy_supportBox_blade = 15;
dy_halfWidthBearing = dy_widthBearing / 2;

% Distance from pulley's applied force on shaft to center front bearing
dy_forcePulley_frontBearing = dy_halfWidthBearing + dy_supportBox_halfFaceWidthPulley; 
% Distance from belt applied force on shaft to center front bearing
dy_blade_frontBearing = dy_supportBox + dy_supportBox_blade - dy_halfWidthBearing;
% Distance between center of bearings
dy_betweenBearings = dy_supportBox - dy_widthBearing;

% Reaction Forces x - Pulley & Blade
% Sum of moments eqn. about z
Rx_rearBearing = -(F_pulley(2) * -dy_forcePulley_frontBearing + F_blade(1) * dy_blade_frontBearing) / dy_betweenBearings;
%fprintf("Rx - Rear Bearing: %.2f N\n", Rx_rearBearing);
Rx_frontBearing = -Rx_rearBearing - F_pulley(2) - F_blade(1);
%fprintf("Rx - Front Bearing: %.2f N\n", Rx_frontBearing);

% Reaction Forces z - Blade & Misalignment
% Sum of moments eqn. about x
% Misaligment Force: 10 N
Rzp_rearBearing = (-F_blade(3) * dy_blade_frontBearing - F_mp(2) * dy_blade_frontBearing) / dy_betweenBearings;
%fprintf("Rzp - Rear Bearing (F_M = 10 N): %.2f N\n", Rzp_rearBearing);

Rzp_frontBearing = -Rzp_rearBearing - F_blade(3);
%fprintf("Rzp - Front Bearing (F_M = 10 N): %.2f N\n", Rzp_frontBearing);

% Misaligment Force: -10 N
Rzn_rearBearing = (-F_blade(3) * dy_blade_frontBearing + F_mp(2) * dy_blade_frontBearing) / dy_betweenBearings;
%fprintf("Rzn - Rear Bearing (F_M = -10 N): %.2f N\n", Rzn_rearBearing);

Rzn_frontBearing = -Rzn_rearBearing - F_blade(3);
%fprintf("Rzn - Front Bearing (F_M = -10 N): %.2f N\n", Rzn_frontBearing);

% Reaction Forces y - Blade Misalignment
% When F_M = 10 N, front bearing reaction force on the retaining ring is -10
% N
Ry_rearBearing_1 = 0;
Ry_frontBearing_1 = -10;

%When F_M = -10 N, rear bearing reaction force on the spacer is 10 N
Ry_rearBearing_2 = 10;
Ry_frontBearing_2 = 0;

% Total Reaction Forces - When F_M = 10 N
R_rearBearing_1 = [Rx_rearBearing, Ry_rearBearing_1, Rzp_rearBearing];
R_frontBearing_1 = [Rx_frontBearing, Ry_frontBearing_1, Rzp_frontBearing];
fprintf("Front Bearing when F_M = 10 N: [%.2fi, %.2fj, %.2fk]\n", Rx_frontBearing, Ry_frontBearing_1, Rzp_frontBearing);
fprintf("Rear Bearing when F_M = 10 N: [%.2fi, %.2fj, %.2fk]\n", Rx_rearBearing, Ry_rearBearing_1, Rzp_rearBearing);

% Total Reaction Forces - When F_M = -10 N
R_rearBearing_2 = [Rx_rearBearing, Ry_rearBearing_2, Rzn_rearBearing];
R_frontBearing_2 = [Rx_frontBearing, Ry_frontBearing_2, Rzn_frontBearing];
fprintf("Front Bearing when F_M = -10 N: [%.2fi, %.2fj, %.2fk]\n", Rx_frontBearing, Ry_frontBearing_2, Rzn_frontBearing);
fprintf("Rear Bearing when F_M = -10 N: [%.2fi, %.2fj, %.2fk]\n", Rx_rearBearing, Ry_rearBearing_2, Rzn_rearBearing);

%% Shaft Design
Mz = (-7243/56.5) * 50-0.889;
T_mm = 1548; % Torque being transmitted
My = T_mm;
Mx = 0; % According to bending moment diagram
FOS_shaft = 1.5;

M_mag = sqrt(Mx^2 + Mz^2); % Bending moments
M_a = M_mag;

hardness_material = 226; % Brinell Scale Hardness

S_ut = 3.4 * hardness_material; % Found in 2-36

% Depends on finish
ka = 3.04 * S_ut^-0.217; % surface factor ka = a*S_ut^b
kb = 1;
kc = 1; % loading factor (remains same)
kd = 1; % temperature factor (remains same)
ke = 0.702; % Reliability of 99.999

S_e_prime = 0.5 * S_ut;
S_e = ka * kb * kc * kd * ke * S_e_prime;

K_f = 5;
K_fs = 3;

shaft_diameter_cubed = (16 * FOS_shaft / pi) * (((2 * K_f * M_a) / S_e) + ((sqrt(3) * K_fs * T_mm) / S_ut));
shaft_diameter = nthroot(shaft_diameter_cubed, 3);
disp(shaft_diameter); % Originally 13.6592 mm

new_kb = (shaft_diameter/7.62)^-0.107;

%% Retaining Ring Dimensions
% 5/8 (15 mm) OD retaining ring
r_t =  0.005 / 0.016;
a_t = 0.039 / 0.016;
groove_diameter = 0.588 * 25.4;
groove_width = 0.039;

K_f = 3.2; % K_f = K_t
K_fs = 2.0; % K_fs = K_ts

sigma_a_prime = K_f * 32 * (-Mz) / (pi * groove_diameter^3);
tau_m_prime_shear = 3 * (K_fs * T_mm * 16 / (pi * groove_diameter^3))^2;
sigma_m_prime_axial = (K_f * 4 * F_mp(2) / (pi * groove_diameter^2))^2;


FOS_shaft = ((sigma_a_prime / S_e) + (sqrt(tau_m_prime_shear + sigma_m_prime_axial) / S_ut))^-1; % Mod. Goodman
disp(FOS_shaft);

S_e_new = ka * new_kb * kc * kd * ke * S_e_prime;
shaft_diameter_cubed = (16 * FOS_shaft / pi) * (((2 * K_f * M_a) / S_e_new) + ((sqrt(3) * K_fs * T_mm) / S_ut));
shaft_diameter = nthroot(shaft_diameter_cubed, 3);
disp(shaft_diameter);

% Shaft Diameter: 5/8 inch, 14.48 mm computed
% n = 2.6290
% Retaining Ring: 5/8 inch outer diameter
%% Pin

n_comp = 1.2;

F_pin = (2 * T_mm) / shaft_diameter;

% d_shaft >> d_pin (factor of 4)
% Currently 5/32 inch Dia.
d_pin = shaft_diameter / 4; 

F_pin_mag = sqrt(F_pin^2 + (F_mp(2)/2)^2);

area_tau_pin = pi * d_pin^2 / 4; 

tau_pin = n_comp * (F_pin_mag / (2 * area_tau_pin));

L_pin = 1 * 25.4; % mm, but adjust in inch

% 4140 Alloy Steel
S_y_pin = 415;

fprintf("h ≥ %.2f mm\n", d_pin);

% Solve for h

%% Key Design

% Maximum Shear Stress Theory
%tau_key_max = S_y_key / (2 * n_comp);

w_key = 3/16 * 25.4;
fprintf("w_key = %.2f should be ≈ %.2f\n", w_key, shaft_diameter / 4);
h_key = 3/16 * 25.4;

S_ut_key = 323; % Found from hardness provided by mcmastercarr
S_y_key = 0.8 * S_ut_key;

area_key = (2 * n_comp * T_mm) / (shaft_diameter * S_y_key); % Minimum key area
% disp(area_key)

l_key = area_key / w_key; % Minimum key length
fprintf("To ensure shear stress dominates, l_key > %.2f\n", w_key);

% Let l_key be 3/4 in 
l_key = 0.75 * 25.4;

%% Set Screw

F_ss = (n_comp * 2 * T_mm) / (shaft_diameter);
disp(F_ss)

% 5/16 inches in length
