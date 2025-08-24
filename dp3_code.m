%% Torque 3 | Omega 3
torque_blade = 1548; % Nmm
omega_bladeRPM = 4600; % RPM
omega_bladeRad = 481.7; % rad/s
%% Torque 2 | Omega 2
torque_shaft = torque_blade;
omega_shaftRPM = 4600;
omega_shaftRad = omega_bladeRad;
%% Gear Ratio
gear_ratio = 1.3333; % Allowable Gear Ratio Range: 1.314 (3500 RPM) to 1.5333 (3000 RPM)

%% Torque 1 | Omega 1
omega_motorRPM = -(1 / gear_ratio) * (omega_shaftRPM); % RPM (3000-3500)
omega_motorRad = omega_motorRPM * 0.104719755; % rad/s
torque_motor = -torque_shaft * gear_ratio;

%% Motor & Shaft Gear Dimensions
module = 2;
d_gear_motor = 80; % mm
d_gear_shaft = 60; 
r_motor = d_gear_motor / 2;
r_shaft = d_gear_shaft / 2;

% Diameter of shaft
d_shaft = 12; % mm

% Diameter of motor, dependent on d_gear_motor
d_motor = 15; % mm

%% Pitch Line Velocity
V_1 = abs(r_motor * omega_motorRad / 1000);

% V_2 = mag(r_gear_shaft * omega_shaft_gearRad / 1000);

%% K_v - Dynamic Stress Concentration Effects
K_v = (3.56 + sqrt(V_1)) / 3.56;

%% Lewis Bending and Contact Stress Formula
S_t_steel = 210; % MPa
S_c_steel = 760; % MPa
pressure_angle = 20;

F_motor = 20; % mm
F_shaft = 19.9; % mm <--- 17.9 mm MINIMUM

Y_motor = 0.3892; % Requires linear interpolation 
Y_shaft = 0.359;

Wt_motor = torque_motor / r_motor;
Wt_shaft = torque_shaft / r_shaft;
Wr_shaft = Wt_shaft*tand(pressure_angle);
W_shaft = Wt_shaft/cosd(pressure_angle);

sigma_lewis_motor = (Wt_motor * K_v) / (F_motor * Y_motor * module);
sigma_lewis_shaft = (Wt_shaft * K_v) / (F_shaft * Y_shaft * module);

C_p = 191;

sigma_term_1 = abs(K_v * Wt_motor / (F_motor * cosd(pressure_angle)));

r_1 = d_gear_motor * sind(pressure_angle) / 2;
r_2 = d_gear_shaft * sind(pressure_angle) / 2;

sigma_term_2 = (1 / r_1  + 1 / r_2);

sigma_contact = -C_p * sqrt((sigma_term_1 * sigma_term_2));

%% Stress Factors and Constants

Ko_shaft = 1.25; % uniform, moderate

Ks_shaft = 1.192 * (F_shaft * sqrt(Y_shaft) * module)^0.0535;

Cmc_shaft = 1; % 1 - uncrowned, 0.8 crowned

if F_shaft / (10 * d_gear_shaft) <= 0.05
    Cpf_shaft = 0.05 - 0.025;
else
    Cpf_shaft = F_shaft / (10 * d_gear_shaft) - 0.025; % F <= 1 in or 25.4 mm
end
Cpm_shaft = 1.1; % should not change

Cma_shaft = (0.247 + 0.0167 * (F_shaft/25.4) + (-0.765e-4) * (F_shaft/25.4)^2); % or use table

Ce_shaft = 1;

KH_shaft = 1 + Cmc_shaft * (Cpf_shaft * Cpm_shaft + Cma_shaft * Ce_shaft);

Yj_shaft = 0.37;

KB_shaft = 1;

%% AGMA Bending Stress
actual_sigma_bending = Wt_shaft * Ko_shaft * K_v * Ks_shaft * (1 / (F_shaft * module)) * KH_shaft * KB_shaft * (1/Yj_shaft);

%% AGMA Safety Factor (Bending)

N_shaft = 20000 * omega_bladeRPM * 60; % bearing hours * blade rpm * 60 min for conversion

YN_shaft = 1.3558 * (N_shaft)^-0.0178;

KT_shaft = 1;

SF_shaft = 210 * YN_shaft * (1 / KT_shaft) * (1 / actual_sigma_bending);

%% AGMA Contact Stress

mG_shaft = d_gear_shaft / d_gear_motor;
mN_shaft = 1; % given at page 758
CH_shaft = 1;

I_shaft = (cosd(pressure_angle) * sind(pressure_angle)) / (2 * mN_shaft) * (mG_shaft / (mG_shaft + 1)); 

actual_sigma_contact = C_p * sqrt(Wt_shaft * Ko_shaft * K_v * Ks_shaft * KH_shaft * (1/d_gear_shaft) * (1/F_shaft) * (1 / I_shaft));

%% AGMA Safety Factor (Contact)
ZN_shaft = 1.4488 * (N_shaft)^-0.023;

SH_shaft = (760 * ZN_shaft * CH_shaft * (1 / KT_shaft) * (1 / actual_sigma_contact)); % squared since stress is linear with transmitted load

% Ks and KH depend on F
