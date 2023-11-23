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

clc
close all
clear


% -------------------------------------------- Import toolboxes

% Selfmade classes


% ******************* Initiate Constants and Dimensions **************** %%

% **************************** Create Robot 

sc_size=1; %m % size of the spacecraft cube

%  Simulation

% -------------------------------------------------- get iiwa7 info

robot = importrobot('iiwa7.urdf');
robot.DataFormat = 'column';

% **************************** Initiate states

q_m=[0;0;0;0;0;0;0];
q_dot_m=[0;0;0;0;0;0;0];
% P0=[0;0;0;0;0;0];
V_I0=[0;0;0;0;0;0];

% End-Effector
% out=sim('EE_iiwa7.slx');

out=sim('EE_iiwa7.slx');
% robot.Bodies{1, 9}.Joint.JointToParentTransform=eye(4); 
% **************************** Prepare properties in workspace                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    o be fed to simulink model

n=7; %number of joints in the manipulator

% rho=transpose([ 0 0 0.15; ...
%                 0 0 0.15+0.19; ...
%                 0 0 0.15+0.19+0.21;... % shoulder joint
%                 0 0 0.15+0.19+0.21+0.19; ...%elbow revolute joint3
%                 0 0 0.15+0.19+0.21+0.19+0.21; ...
%                 0 0.06070 0.15+0.19+0.21+0.19+0.21+0.19; ...
%                 0 0 0.15+0.19+0.21+0.19+0.21+0.19+0.081] ... %wrist joint
%                         ); %m %position of joints in initial configuration

% rho=transpose([0 0 0.025; 0 0 0.025; 0 0 0.025;... % shoulder ball joint
%             0 0 4.5+0.025; ... %elbow revolute joint3
%             0 0 9+0.025; 0 0 9+0.025; 0 0 9+0.025] ... %wrist ball joint
%             ); %m %position of joints in initial configuration

w=transpose([   0 0 1; ...
                0 1 0; ...
                0 0 1; ... % shoulder ball joint
                0 -1 0;... %elbow 
                0 0 1; ...
                0 1 0; ...
                0 0 1  ... %wrist joint
                            ]); %vector of rotation of each joint in inertial frame in initial pose

% **************************** Initiate Iota the inclusion map of the base
iota0=eye(6,6);

% **************************** Joint positions for internal
rho(1:3,1)=out.R.signal1.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,2)=out.R.signal2.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,3)=out.R.signal3.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,4)=out.R.signal4.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,5)=out.R.signal5.data(1:3,4,1);%+[0;0;0.5*sc_size];
rho(1:3,6)=out.R.signal6.data(1:3,4,1);%+[0;0;0.5*sc_size]; rho(2,6)=0;
rho(1:3,7)=out.R.signal7.data(1:3,4,1);%+[0;0;0.5*sc_size];

% **************************** Joint positions for coupled
% rho(1:3,1)=out.R.signal1.data(1:3,4,1)+[0;0;0.5*sc_size];
% rho(1:3,2)=out.R.signal2.data(1:3,4,1)+[0;0;0.5*sc_size];
% rho(1:3,3)=out.R.signal3.data(1:3,4,1)+[0;0;0.5*sc_size];
% rho(1:3,4)=out.R.signal4.data(1:3,4,1)+[0;0;0.5*sc_size];
% rho(1:3,5)=out.R.signal5.data(1:3,4,1)+[0;0;0.5*sc_size];
% rho(1:3,6)=out.R.signal6.data(1:3,4,1)+[0;0;0.5*sc_size]; rho(2,6)=0;
% rho(1:3,7)=out.R.signal7.data(1:3,4,1)+[0;0;0.5*sc_size];

% Forward
% temp=rho(1:3,1);
% for i=1:7
%     rho(1:3,i)=rho(1:3,i)-temp;
% end


% **************************** form the overall twist matrix
Xi_m_matrix=zeros(6*n,n);
for i=1:n
    v(1:3,i)=-cross(w(1:3,i),rho(1:3,i));
    Xi_m_temp(1:6,i)=[v(1:3,i);w(1:3,i)];
    Xi_m_matrix(6*(i-1)+1:6*(i-1)+6,i)=[v(1:3,i);w(1:3,i)];
end

Xi_m(:,:)=Xi_m_temp;


% **************************** Set the initial poses relative to spacecraft
R(1:3,1:3,1)=out.R.signal1.data(1:3,1:3,1);
R(1:3,1:3,2)=out.R.signal2.data(1:3,1:3,1);
R(1:3,1:3,3)=out.R.signal3.data(1:3,1:3,1);
R(1:3,1:3,4)=out.R.signal4.data(1:3,1:3,1);
R(1:3,1:3,5)=out.R.signal5.data(1:3,1:3,1);
R(1:3,1:3,6)=out.R.signal6.data(1:3,1:3,1);
R(1:3,1:3,7)=out.R.signal7.data(1:3,1:3,1);
% R(1:3,1:3,8)=out.R.signal8.data(1:3,1:3,1);

for i=1:n
    g_bar(1:4,1:4,i)=[R(1:3,1:3,i) rho(1:3,i); 0 0 0 1];
end

R_bar_rel(1:3,1:3,1)=g_bar(1:3,1:3,1);
for i=2:n
    R_bar_rel(1:3,1:3,i)=inv(g_bar(1:3,1:3,i-1))*g_bar(1:3,1:3,i);
end

% Set the initial poses of CoM of bodies in joint frames
g_cm(1:4,1:4,1)=[R_bar_rel(:,:,1) rho(:,1); 0 0 0 1]; % ee
for i=2:n
    g_cm(1:4,1:4,i)=[R_bar_rel(:,:,i) rho(:,i)-rho(:,i-1); 0 0 0 1];
end

g_cm(1:4,1:4,n)=[eye(3) [0; 0; -sc_size/2]; 0 0 0 1]; %spacecraft

% g_cm(4,1:3,n)=[0 0 -sc_size/2];

% Set the Adjoint of initial poses of CoM of bodies in joint frames
for i=1:n
    Ad_gcm_inv(:,:,i)=inv(Adjoint(g_cm(:,:,i)));
    Ad_gbar_inv(:,:,i)=inv(Adjoint(g_bar(:,:,i)));
end

%Xi_m=0;
% mu=zeros(6,1)';
% mu_t=zeros(6,1)';

% End-Efffector
m0=robot.Bodies{1,n+1}.Mass;

% arm + spacecraft in reverse
mm=zeros(1,n);
mm(n)=10;%kg spacecraft

for i=2:n
    mm(n+1-i)=robot.Bodies{1,i}.Mass;
end

% ----------------- inertia matrices
I0(1:3,1:3)=[robot.Bodies{1,8}.Inertia(1) robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(5); ...
    robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(2) robot.Bodies{1,8}.Inertia(4); ...
    robot.Bodies{1,8}.Inertia(5) robot.Bodies{1,8}.Inertia(4) robot.Bodies{1,8}.Inertia(3)];

%spacecraft
Im(1:3,1:3,n)=eye(3)*1.66667; %for cube with density 0.8
%arm
for i=2:n
    Im(1:3,1:3,n-i+1)=[robot.Bodies{1,i}.Inertia(1) robot.Bodies{1,i}.Inertia(6) robot.Bodies{1,i}.Inertia(5); ...
    robot.Bodies{1,i}.Inertia(6) robot.Bodies{1,i}.Inertia(2) robot.Bodies{1,i}.Inertia(4); ...
    robot.Bodies{1,i}.Inertia(5) robot.Bodies{1,i}.Inertia(4) robot.Bodies{1,i}.Inertia(3)];
end


% calculate Inertia matrices in the joint frames
[M_curly0,M_curlym]=M_curly(m0,I0,mm,Im,Ad_gcm_inv);

% ************** Initiate forces

% f0=[0;0;0;0;0;0];
% f0=[0;0;0;0;0;0;0];
f_0=[0;0;0;0;0;0];
f_m=[0;0;0;0;0;0;0];
f_e=[0;0;0;0;0;0];



% Form the math matrix diagonalized in the base frame

[M_frak0,M_frak] =M_frak(M_curly0,M_curlym,Ad_gbar_inv);

diag_M =diagonalize(M_frak0, M_frak);

% Test and Validation

% M0_temp=sim('M0_initiate',0.01);
% M0=M0_temp.M0.data(:,:,1);

% P0=M_frak0*V_I0;

% ----------------- inertia matrices for simscape model
temp=Ad_gcm_inv(:,:,1)'*[m0*eye(3) zeros(3);zeros(3) [robot.Bodies{1,8}.Inertia(1) robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(5); ...
    robot.Bodies{1,8}.Inertia(6) robot.Bodies{1,8}.Inertia(2) robot.Bodies{1,8}.Inertia(4); ...
    robot.Bodies{1,8}.Inertia(5) robot.Bodies{1,8}.Inertia(4) robot.Bodies{1,8}.Inertia(3)]]*Ad_gcm_inv(:,:,1);
I0(1:3,1:3)=temp(4:6,4:6);

%spacecraft 
temp=Ad_gcm_inv(:,:,n)'*[mm(n)*eye(3) zeros(3); zeros(3) Im(1:3,1:3,n)]*Ad_gcm_inv(:,:,n);
Im(1:3,1:3,n)=temp(4:6,4:6);
%arm
for i=2:n
    Im(1:3,1:3,n-i+1)=inv(R_bar_rel(:,:,n-i+1))'*[robot.Bodies{1,i}.Inertia(1) robot.Bodies{1,i}.Inertia(6) robot.Bodies{1,i}.Inertia(5); ...
    robot.Bodies{1,i}.Inertia(6) robot.Bodies{1,i}.Inertia(2) robot.Bodies{1,i}.Inertia(4); ...
    robot.Bodies{1,i}.Inertia(5) robot.Bodies{1,i}.Inertia(4) robot.Bodies{1,i}.Inertia(3)]*inv(R_bar_rel(:,:,i-1));
end



%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! UPDATE
rho_0t=[0.25;0.25;0.26]; %m
V_0t=[0;0;0];%[-0.0005;-0.0005;0];
%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

mu=zeros(6,1);
mu_t=zeros(6,1);

% m0=0.5; %base
% me=50;% wrist, kg
% mm=[2 2 2 2 50 50 50]; % link1, link2, base

% Inertia of arm
% Im=zeros(3,3,n);
% for i=1:n
%     Im(1:3,1:3,i)=eye(3);
% end

% I0=eye(3)*0.000208333; %for cube with density 0.8
% Ie=eye(3)*33.3333; %for cube with density 0.8

g_I0=eye(4);

% *************************** Set Target parameters

% w_t=[0 0.05 0.05];
rho_0t=[0.23;0.27;0.26]; %m
V_0t=[-0.002;-0.002;0];%[-0.0005;-0.0005;0];
w_t=[0;0;-0.01];
mt=10;
M_t=[eye(3)*mt zeros(3); zeros(3) eye(3)*80];

mu_t=M_t*[V_0t;w_t];

% *************************** Set Controller parameters

K_p=[0.1 0 0 0 0 0;0 0.1 0 0 0 0; 0 0 0.1 0 0 0; 0 0 0 0.01 0 0;
    0 0 0 0 0.01 0;0 0 0 0 0 0.01];%0.05*eye(6);
K_d=[0.7*eye(3) zeros(3);zeros(3) 0.5*eye(3)];
K_i=[0.01*eye(3) zeros(3);zeros(3) 0.001*eye(3)];


%
% M0_temp=sim('M0_initiate',0.01);
% M0=M0_temp.M0(:,:,1);

% remove fixed base
% removeBody(robot,'iiwa_link_0');

% P=M0*V_I0+M0\M0m*q_dot_m; mu=P;
q_m=[0.2;1.2;0.3;1.8;0.5;1.3;0.6];
% q_dot_m=0.1*[0.2;1;0.3;1;0.5;0;0];%[0;0;0;0;0;0;0];
f_m=[0;0;0;0;0;0;0];

M_temp=sim('M0_initiate',0.01);
M0=M_temp.M0.data(:,:,1);%iota0'*M_frak0*iota0;
M0m=M_temp.M0m.data(:,:,1);
% M0m=M_temp.M0m.data(:,:,1);
P0=M0*V_I0+M0\M0m*q_dot_m;
mu=P0;

%% ********************** Simulink Controller

q_m=[0.5;0.3;0.6;1;0.9;0.8;0.9];
q_dot_m=[0;0.1;0.2;0;0;0;0];
f_m=[0;0;0;0;0;0;0];
% V_I0=M0\P-A_connection*q_dot_m;
V_I0=[0;0;0;0;0;0];
M_temp=sim('M0_initiate',0.01);
M0=M_temp.M0.data(:,:,1);
M0m=M_temp.M0m.data(:,:,1);
P=M0*V_I0+M0\M0m*q_dot_m; mu=P;

%%
out=sim('test_Dynamics_coupled_full',0.01);
format long
qddot=out.qddot.data(:,1);
qddot_ref=out.qddot_ref.data(1,:);
V=out.V.data(:,1);
V_ref=out.V_ref.data(1,:);
%                 ['q=',num2str(q_m1),',',num2str(q_m2),' & ', 'q_dot=',num2str(q_dot_m1),',',num2str(q_dot_m2),' & ', 'q_ddot=',num2str(qddot(:,1)'),' & q_ddot_ref=',num2str(qddot_ref(1,:))]
table(q_m1,q_m2,V_I01,V_I02,V_I03,qddot(:,1)'-qddot_ref(1,:),V'-V_ref)

sim('EE_Controller_v3.slx') 

%% ************* Symbolic Spacecraft-Manipulator System *********** %%

% Simulation
% robot.simulate(ts=ts, tf=tf, dt=dt, rec=rec);

Jacobian=Jacobian(robot,q);

Ad_g0I=simplify(Adjoint(robot.vehicle.g));


% Masses
robot.vehicle.M_frak=[robot.vehicle.mass zeros(3); zeros(3) robot.vehicle.inertia];
for j=1 : robot.n
    robot.links(j).M_frak=transpose(inv(Adjoint(robot.links(j).g_cm0)))*[robot.links(j).mass zeros(3); zeros(3) robot.links(j).inertia]*inv(Adjoint(robot.links(j).g_cm0));
    robot.links(j).M_frak=simplify(robot.links(j).M_frak);
end

% for loop
for j=1:robot.n
    Xi_m((j-1)*6+1:j*6,j)=robot.joints(j).xi;%[robot.joints(1).xi zeros(6,1);zeros(6,1) robot.joints(2).xi];
end
Xi_m=Xi_m(1:robot.n*6,1:robot.n);


Ad_g10=simplify(Adjoint(g(robot.joints(1).xi,-q1)));
Ad_g21=simplify(Adjoint(g(robot.joints(2).xi,-q2)));
Ad_g20=Ad_g21*Ad_g10;

[Lm0,Lm]=form_L(robot,q);


Lm02=[Ad_g10;Ad_g20];
Lm=[eye(6) zeros(6); Ad_g21 eye(6)];

% Calculate the Mass matrix

for j=1:robot.n
    diagM((j-1)*6+1:j*6,(j-1)*6+1:j*6)=robot.links(j).M_frak;
end

M0=simplify(transpose(robot.vehicle.iota_0)*robot.vehicle.M_frak*robot.vehicle.iota_0...
    +transpose(robot.vehicle.iota_0)*transpose(Lm0)*diagM*Lm0*robot.vehicle.iota_0);

M0m=simplify(transpose(robot.vehicle.iota_0)*transpose(Lm0)*diagM*Lm*Xi_m);

Mm=simplify(transpose(Xi_m)*transpose(Lm)*diagM*Lm*Xi_m);


% Calculate the connection

A=simplify(inv(M0)*M0m);

% Find generalized Mas Matrix

M_hat=Mm-transpose(A)*M0*A;

% Find generalized Momentum

Omega=V_curly+A*qdot;
P=M0*Omega;

% Calculate ad*
ad_V_curly=lie_alg(V,robot.vehicle.iota_0);

% Calculate P_dot
P_dot=ad_V_curly*P;

%

%% ************* Propagate the Spacecraft-Manipulator System *********** %%

% __________ Calculate the tranformation matrix __________ %
% _ for each joint and the corresponding CG of the body __ %
% for i=1:robot.n
%     % Initial g including Rotation and Translation caused by joint i
%     robot.links(i).T = g_Matrix(0,robot.joints(i).T(1:3,4),xi(:,:,i));
%     % Initial Homogeneous transformation of the joints
%     if i~=1
%         robot.joints(i).T=[eye(3),robot.joints(i-1).T(1:3,4)/2;zeros(1,3),1];
%     else
%         robot.joints(i).T=[eye(3),r0;zeros(1,3),1];
%     end
% end

time_step=0.01;
tspan=10;
step=0;

y=zeros(2*(b+n),tspan/time_step);
y(1:2*(n+b),1)=[q;qdot_0];
% qm=[0;0]; 
tt=[0:time_step:tspan];

for time=0:time_step:tspan
    step=step+1;
    
    % extract V_0 and q_m and their derivatives
    P=y(1:b,step); %V_0I_curly
    qm=y(b+1:b+n,step);
    P_dot=y(b+n+1:2*b+n,step);
    qm_dot=y(2*b+n+1:2*(b+n),step);
%     R0=Rotation(wb4,qb(4))*Rotation(wb5,qb(5))*Rotation(wb6,qb(6));
%     r0=qb(1:3);
    g10=g(robot.joints(1).xi,q(1));
    g21=g(robot.joints(2).xi,q(2));
    % **************************** Kinematics *************************** %
    
    % __________ Kinematics __________ %
    [Rij,R_cm,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot,xi);
    %End-Effector
    g_n=[Rij(1:3,1:3,end),rJ(1:3,end);zeros(1,3),1]*[eye(3),[0;L2;0];zeros(1,3),1];
    ree(:,step)=g_n(4,1:3);
    % __________ Differential kinematics __________ %
    [t0,tm,Tij,Ti0,P0,pm]=DifferentialKinematics(R0,r0,rL,e,g,qb_dot,qm_dot,robot);

    %End-effector Jacobian
    [J0ee, Jmee]=Jacob(g_n(1:3,4),r0,rL,P0,pm,robot.n,robot);

    % __________ Inertia Matrices __________ %
    % The conjugate mapping and unification of the frame of representation of
    % inertia and mass matrices
    % Inertias in inertial frames
    [I0,Im]=I_I(R0,R_cm,robot);
    % Mass matrix
    [M0_tilde,Mm_tilde]=Mass_Matrix(I0,Im,Tij,Ti0,robot);


    %Generalized Inertia matrix
    [H0, H0m, Hm] = Hgen(M0_tilde,Mm_tilde,Tij,Ti0,P0,pm,robot);
    H=[H0,H0m;H0m',Hm];
    % C matrix (Very uncertain, therefor not used)
    [C0, C0m, Cm0, Cm] = Cgen(t0,tm,I0,Im,M0_tilde,Mm_tilde,Tij,Ti0,P0,pm,robot);
    C=[C0,C0m;Cm0,Cm];
    
    % ***************************** Dynamics **************************** %

    % __________ Forward Dynamics __________ %

    %External forces (includes gravity and assumes z is the vertical direction)
    F0=[0;0;0;0;0;-robot.spacecraft.mass*gg]; % in generalized coord
    Fm=[zeros(5,robot.n_q); -robot.links(1).mass*gg,-robot.links(2).mass*gg];

    %Joint torques (not external torques assumed)
    tauq0=zeros(6,1);
%     tauqm=zeros(robot.n_q,1);
    tauqm=[0;0];

    %Forward Dynamics (get the velocities of generalized coordinates)
    [u0dot_FD,umdot_FD] = Forward_Dynamics(tauq0,tauqm,F0,Fm,t0,tm,P0,pm,I0,Im,Tij,Ti0,qb_dot_0,qm_dot_0,robot);
    
      % Update generalized coordinates
%     qb_dot=u0dot_FD; % Initial Base-spacecraft linear and angular velocities
%     qm_dot=umdot_FD; % Initial Joint velocities [Rad]
    
%     qb=qb+qb_dot*time_step;
%     qm=qm+qm_dot*time_step;
%     q=[qb;qm]

    dy=y(n+7:2*n+12,step);
    ddy=-inv(H)*C*y(n+7:2*n+12,step)+inv(H)*[u0dot_FD;umdot_FD];
    
    
    y(1:n+6,step+1)=y(1:n+6,step)+dy*time_step;
    y(n+7:2*n+12,step+1)=y(n+7:2*n+12,step)+ddy*time_step;
    
%     opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
%     [t,y] = ode45(Dynamics,tspan,y0,opts);
    Momentum(1:n+6,step)=H*y(n+7:2*n+12,step);
end


%% ----------------------------------- Results and Visualization

figure(1)
% plot3(y(1,:),y(2,:),y(3,:))
plot(tt(1:100),y(1,1:100))
hold on
plot(tt(1:100),y(2,1:100))
plot(tt(1:100),y(3,1:100))
legend x y z

figure(2)
plot(tt(1:100),y(4,1:100))
hold on
plot(tt(1:100),y(5,1:100))
plot(tt(1:100),y(6,1:100))
legend \theta_{b1} \theta_{b2} \theta_{b3}

figure(3)
plot(tt(1:100),y(7,1:100))
hold on
plot(tt(1:100),y(8,1:100))
legend \theta_1 \theta_2

figure(4)
plot3(y(1,1:100),y(2,1:100),y(3,1:100))
legend r_{b}
grid on

figure(5)
% plot3(y(1,:),y(2,:),y(3,:))
plot(tt(1:100),y(9,1:100))
hold on
plot(tt(1:100),y(10,1:100))
plot(tt(1:100),y(11,1:100))
legend v_x v_y v_z

figure(6)
plot(tt(1:100),y(12,1:100))
hold on
plot(tt(1:100),y(13,1:100))
plot(tt(1:100),y(14,1:100))
legend \omega_{b1} \omega_{b2} \omega_{b3}

figure(7)
plot(tt(1:100),y(15,1:100))
hold on
plot(tt(1:100),y(16,1:100))
legend \omega_1 \omega_2
counter=1;

%% M matrix verification in internal
counter=1;

for q_m1=[1]
    for q_m2=[1.5]
        for q_m3=[1]
            for q_m4=[1.5]
                for q_m5=[1]
                    for q_m6=[1.5]
                        for q_m7=[1]
        % ------------------
        for fm1=[0 0.05]
            for fm2=[0 0.1]
                for fm3=[0 0.05]
                    for fm4=[0 0.01]
                        for fm5=[0 0.05]
                            for fm6=[0 0.01]
                                for fm7=[0 0.05]
                                q_m=[q_m1;q_m2;q_m3;q_m4;q_m5;q_m6;q_m7];
                                q_dot_m=[0;0;0;0;0;0;0];
                                f_m=[fm1;fm2;fm3;fm4;fm5;fm6;fm7];
%                                 V_I0=M0\P-A_connection*q_dot_m;
                                M_temp=sim('M0_initiate',0.01);
                                M0=M_temp.M0.data(:,:,1);
                                M0m=M_temp.M0m.data(:,:,1);
%                                 P=M0*V_I0+M0m*q_dot_m;
                                out=sim('KUKA_Dynamics_internal_massonly.slx',0.01);
                                format long
                                qerror=out.qerror.data(:,1);
%                                 qddot_ref=out.qddot_ref.data(1,:);
%                                 ['q=',num2str(q_m1),',',num2str(q_m2),' & ', 'q_dot=',num2str(q_dot_m1),',',num2str(q_dot_m2),' & ', 'q_ddot=',num2str(qddot(:,1)'),' & q_ddot_ref=',num2str(qddot_ref(1,:))]
%                                 qerror(:,1)'
                                results(counter,:)=qerror(:,1)';
                                counter=counter+1;
                                end
                            end
                        end
                    end
                end
            end
        end
        % ------------------
                        end
                    end
                end
            end
        end
    end
end

%% C matrix verification in internal

counter=1;
f_m=[0;0;0;0;0;0;0];

for q_m1=[1]
    for q_m2=[1.5]
        for q_m3=[1]
            for q_m4=[1.5]
                for q_m5=[1]
                    for q_m6=[1.5]
                        for q_m7=[1]
        % ------------------
        for qdot1=[0.02 0.05]
            for qdot2=[0.05 0.1]
                for qdot3=[0.01 0.05]
                    for qdot4=[0.02 0.01]
                        for qdot5=[0.01 0.05]
                            for qdot6=[0.02 0.01]
                                for qdot7=[0.01 0.02]
                                q_m=[q_m1;q_m2;q_m3;q_m4;q_m5;q_m6;q_m7];
                                q_dot_m=[qdot1;qdot2;qdot3;qdot4;qdot5;qdot6;qdot7];
                                
%                                 V_I0=M0\P-A_connection*q_dot_m;
                                M_temp=sim('M0_initiate',0.01);
                                M0=M_temp.M0.data(:,:,1);
                                M0m=M_temp.M0m.data(:,:,1);
%                                 P=M0*V_I0+M0m*q_dot_m;
                                out=sim('KUKA_Dynamics_internal_MC.slx',0.01);
                                format long
%                                 qerror=out.qerror.data(:,1);
%                                 qddot_ref=out.qddot_ref.data(1,:);
%                                 ['q=',num2str(q_m1),',',num2str(q_m2),' & ', 'q_dot=',num2str(q_dot_m1),',',num2str(q_dot_m2),' & ', 'q_ddot=',num2str(qddot(:,1)'),' & q_ddot_ref=',num2str(qddot_ref(1,:))]
                                results(counter,:)=out.qerror.data(:,:,1)';
                                end
                            end
                        end
                    end
                end
            end
        end
        % ------------------
                        end
                    end
                end
            end
        end
    end
end

