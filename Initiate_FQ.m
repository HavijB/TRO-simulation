% ************************************************************************%
%                                                                         %
%                   Relative output regulation of                         %
%                   space manipulators on Lie groups                      %
%                                                                         %
%                                                                         %
% Developed by:     Borna Monazzah Moghaddam                              %
%                   Autonomous Space Robotics and Mechatronics Laboratory %
% Supervised by:    Robin Chhabra,                                        %
%                   Carleton University, Ottawa, Canada.                  %
%                                                                         %
% Initiated: 2022 August                                                  %
%                                                                         %
% Edited:                                                                 %
% ************************************************************************%

% -------------------------------------------- Import toolboxes

% Selfmade classes


% *********************** Initiate Forces & States **************** %%

% ***************************** Create Robot **************************** %

% **************************** Initiate states

q_m=[0.2;1.5;0.3;1.8;0.5;0.3;0.6]; %q_m=[0.2;1.2;0.3;1.8;0.5;1.3;0.6];
q_dot_m=[0;0;0;0;0;0;0];% q_dot_m=0.1*[0.2;1;0.3;1;0.5;0;0];
V_I0=[0;0;0;0;0;0];

g_I0=eye(4);

% **************************** Prepare properties in workspace                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    o be fed to simulink model

n=7; %number of joints in the manipulator

% ************** Initiate forces

f_0=[0;0;0;0;0;0];
f_m=[0;0.1;0.2;0;0;0.01;0];
f_e=[0;0;0;0;0;0];

% *************************** Set Target parameters

% rho_0t=[0.24;0.26;0.25]; %m
% V_0t=[-0.01;-0.01;0];%[-0.0005;-0.0005;0];
% w_t=[0;0;-0.05];
rho_0t=[0.25;0.46;0.25]; %rho_0t=[0.25;0.26;0.25];
V_0t=[0;0;0];%[-0.0005;-0.0005;0];
w_t=[0;0;0.01];

mt=10;
M_t=[eye(3)*mt zeros(3); zeros(3) eye(3)*0.416667];

mu_t=M_t*[V_0t;w_t];

% *************************** Set Controller parameters

K_p=50*[0.1 0 0 0 0 0;0 0.1 0 0 0 0; 0 0 0.1 0 0 0; 0 0 0 0.01 0 0;
    0 0 0 0 0.01 0;0 0 0 0 0 0.01];%0.05*eye(6);
K_d=10*[1.3*eye(3) zeros(3);zeros(3) 0.8*eye(3)];
K_i=5*[0.1*eye(3) zeros(3);zeros(3) 0.01*eye(3)];


% P=M0*V_I0+M0\M0m*q_dot_m; mu=P;

% f_m=[0;0;0;0;0;0;0];

M_temp=sim('M0_initiate',0.01);
M0=M_temp.M0.data(:,:,1);%iota0'*M_frak0*iota0;
M0m=M_temp.M0m.data(:,:,1);
% M0m=M_temp.M0m.data(:,:,1);
P0=M0*V_I0+M0\M0m*q_dot_m;
mu=P0;