function M_curly= M_curly(m0,I0,mm,Im,Ad_gm_inv)
% Calculates the Inertia matrix in the joint frames

    M_curly0=transpose(Ad_gm_inv(i,:,:)*)

% form set of M in the joint frames
M_curly0(:,:)=[m0*eye(3,3);zero(3,3);zeros(3,3);I0];
for i=1:n
    M_curly(i,:,:)=[mm(i)*eye(3,3);zero(3,3);zeros(3,3);II(i,:,:)];
end
