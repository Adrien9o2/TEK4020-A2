a_1 = 1;
a_2 = 1;
l_1 = 0.5;
l_2 = 0.5;
m_l_1 = 50;
m_l_2 = 50;
I_l_1 = 10;
I_l_2 = 10;
k_r_1 = 100;
k_r_2 = 100;
m_m_1 = 5;
m_m_2 = 5;
I_m_1 = 0.01;
I_m_2 = 0.01;

F = [0.1, 0;
    0, 0.1];

gravity = 9.81;

f = 0.5;

% kp_1 = 400.0; %simple PD comp
% kp_2 = 400.0;
% kd_1 = 600.0;
% kd_2 = 500.0;

% kp_1 = 500; %Compliance
% kp_2 = 500;
% kd_1 = 10.0;
% kd_2 = 10.0;

kp_1 = 2500; %Impedance
kp_2 = 2500;
kd_1 = 500.0;
kd_2 = 500.0;
md_1 = 100;
md_2 = 100;

%force cntrl
kf_1 = 0.05;
kf_2 = 0.05;
f_target = [5;0];

%Assignement 2

xr = 0.5;
kx = 100;

xd_1 = 0.75;
xd_2 = 0.25;

init = [0.75*pi;-0.75*pi];

