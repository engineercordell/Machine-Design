% Force: N
% Dimension: mm

r_o = 50.8;
d_o = r_o * 2;
t = 9.525;
r_i = r_o - t;
d_i = r_i * 2;
area = pi*(r_o^2-r_i^2);

pin_D = 9.525;
pin_r = pin_D / 2;
MoI_pin = (pi*pin_D^4) / 64;
MoI_arm = (pi/64)*(d_o^4 - d_i^4);

% Polar Moment of Inertia of Pin
polar_MoI_pin = (pi*pin_D^4)/32;

% Polar Moment of Inertia of Pivot Arm
polar_MoI = (pi/32)*(d_o^4 - d_i^4);

w_pivot_arm = 13.5465;

length_shaft = 165;

torque_blade = 1545.63;
torque_motor = 2060.84;
origin_shaft_length_x = 410;
arc_length = 414.5;

r_c = 832.625;
r_n = r_o^2 / (2 * (r_c - sqrt(r_c^2 - r_o^2)));
e = r_c - r_n;

origin_actuator_length_x = 205;
origin_mounting_length_z = 150;
centershaft_blade_length_y = 65;
centershaft_pulley_length_y = 100;
origin_actuator_length_z = 25;

driving_pulley_radius = 50.8;
driven_pulley_radius = 38.1;
blade_radius = 101.6;

w_support_box = -19.6;
f_cutting_z = 40;

diff_driving_driven_pulley_radius = driving_pulley_radius - driven_pulley_radius;

% Angle of the belt with respect to the origin
angle_pulley = atand(diff_driving_driven_pulley_radius / origin_shaft_length_x);

f_slack_mag = 45;
f_slack_vec = [f_slack_mag*cosd(angle_pulley), 0, f_slack_mag*sind(angle_pulley)];

% Angle of pulley with respect to global axis
f_pulley_tension_diff_mag = (f_slack_mag*driving_pulley_radius*cosd(angle_pulley) + torque_motor) / (driving_pulley_radius * cosd(angle_pulley)); 
f_pulley_tension_diff_vec = [f_pulley_tension_diff_mag*cosd(angle_pulley), 0, -f_pulley_tension_diff_mag*sind(angle_pulley)];

% Belt force acts in +x direction -z direction
f_belt = f_slack_vec + f_pulley_tension_diff_vec;

% % Reaction force on blade from material acts in +x direction
% Found by taking moment about shaft to yield cutting reaction force
% **Might need to include moments about pulley
f_reaction_blade_x = (torque_blade / blade_radius);

% Force x on shaft w/o cutting reaction force
f_shaft(1) = f_belt(1) + f_reaction_blade_x;

% Force z on shaft from force by pulley, cutting force, and support box
% weight.
f_shaft(3) = f_belt(3) + f_cutting_z + w_support_box;

% Moment on shaft
moment_shaft = [f_cutting_z*centershaft_blade_length_y + f_belt(3)*centershaft_pulley_length_y, 0, f_belt(1)*centershaft_pulley_length_y + f_reaction_blade_x*centershaft_blade_length_y];

% Actuator forces (sum of moment about pivot pin eqn)
angle_rotated_pivotarm_tangent_x = atand(origin_actuator_length_z/origin_actuator_length_x);
angle_actuator = 40.5;
f_actuator_mag = (-f_shaft(3)*origin_shaft_length_x)/(sind(angle_actuator)*origin_actuator_length_x + cosd(angle_actuator)*origin_actuator_length_z);
f_actuator_vec = [f_actuator_mag*cosd(angle_actuator), 0, f_actuator_mag*sind(angle_actuator)];

% Pivot forces (sum of forces from actuator, shaft, weight of pivot arm)
f_pivot_vec = [-f_shaft(1) + (-f_actuator_vec(1)), 0, -f_shaft(3) + (-f_actuator_vec(3))];

% Pivot Pin Moment y
m_Ax = f_actuator_vec(1)*origin_actuator_length_z;
m_Az = -f_actuator_vec(3)*origin_actuator_length_x;
m_weight = w_pivot_arm*origin_actuator_length_x;
m_Sz = -f_shaft(3)*origin_shaft_length_x;
moment_pin_arm = [0, m_Ax + m_Az + m_weight + m_Sz, 0];

% Moments Acting on Pivot Pin
m_tangent_cutting_force = cross([origin_shaft_length_x, centershaft_blade_length_y, -blade_radius], [f_reaction_blade_x, 0, 0]);
m_vertical_cutting_force = cross([origin_shaft_length_x, centershaft_blade_length_y, -blade_radius], [0, 0, f_cutting_z]);
m_pulley = cross([origin_shaft_length_x, -centershaft_pulley_length_y, driven_pulley_radius], f_belt);

% Net Moment in Pin
net_moment_pin = moment_pin_arm + m_tangent_cutting_force + m_vertical_cutting_force + m_pulley;

% Peak Axial Load
f_peak_axial = 146.987;

% Moments Acting on Pivot Arm
m_tangent_cutting_force_pivot_arm = cross([origin_actuator_length_x, centershaft_blade_length_y, -blade_radius], [f_reaction_blade_x, 0, 0]);
m_vertical_cutting_force_pivot_arm = cross([origin_actuator_length_x, centershaft_blade_length_y, -blade_radius], [0, 0, f_cutting_z]);
m_pulley_pivot_arm = cross([origin_actuator_length_x, -centershaft_pulley_length_y, driven_pulley_radius], f_belt);

% Net Moment in Pivot Arm
net_moment_pivot_arm = m_tangent_cutting_force_pivot_arm + m_vertical_cutting_force_pivot_arm + m_pulley_pivot_arm;

% Shear Force in Pivot Arm
shear_force_pivot_actuator = (f_shaft(3)*origin_actuator_length_x)/origin_actuator_length_x;
shear_force_actuator_shaft = w_pivot_arm + f_shaft(3);

c_o = r_o - r_n;

% Bending Moment in Pivot Arm
bending_moment_pivot_arm = sqrt(net_moment_pivot_arm(2)^2 + net_moment_pivot_arm(3)^2);

% Maximum Bending Stress in Pivot Arm
bending_stress_pivot_arm = -(bending_moment_pivot_arm * c_o)/(area*e*r_o);

% Torsional Shear
torsional_shear = (net_moment_pivot_arm(1) / polar_MoI)*(r_o);
% Von Mises in Pivot Arm
von_mises_arm = sqrt(bending_stress_pivot_arm^2+3*torsional_shear^2);

% Torsional Shear for Pin
torsional_shear_pin = (net_moment_pin(1) / polar_MoI_pin)*(pin_r);
% Bending Moment Pin
bending_moment_pin = sqrt(net_moment_pin(2)^2 + net_moment_pin(3));
% Maximum Bending Stress for Pin
bending_stress_pin = (bending_moment_pin * pin_r) / (MoI_pin);
% Von Mises in Pin
von_mises_pin = sqrt(bending_stress_pin^2+3*torsional_shear_pin^2);

% Deflection about y and z
def_y = (net_moment_pin(2)*(origin_shaft_length_x)^2)/(2*(69000)*(MoI_arm));
def_z = (net_moment_pin(3)*(origin_shaft_length_x)^2)/(2*(69000)*(MoI_arm));

% Deflection magnitude
def_mag = sqrt(def_y^2+def_z^2);





