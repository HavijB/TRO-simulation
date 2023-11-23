n=length(q_m);

% Initiate
J=zeros(6,6 + n);
% Jm=zeros(6,robot.n);

% Vehicle portion
J0=eye(6,6);
J(1:6,1:6)=J0;

% ____________ Manipulator Jacobian _____________ %

% Initiate
j=1;
Jm=zeros(6,n);

Jm(1:6,j)=Xi_m(1:6,j);
for j=2:n %Iterate through all joints
    for i=1:j-1
        temp=eye(6);
        temp=temp*Adjoint(g_Matrix(Xi_m(:,i),q_m(i)));
    end
    Jm(1:6,j)=temp*Xi_m(:,j);
end


J_e0=Adjoint(g_Ie)\iota0; %eq67
J_em=Adjoint(g_Ie)\Jm; % transform to inertia

% J_e0=J_eeI(1:6,1:6);
% J_em=J_eeI(1:6,7:n);