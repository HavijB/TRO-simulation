function [M_curly0, M_curly]= M_curly_ee(m0,I0,mm,Im,Ad_gbar_rel)
% Calculates the Inertia matrix in the joint frames
n=length(mm);

M_curly=zeros(n,6,6);
% M_curly0=transpose(Ad_gm_inv(i,:,:)*)

% form set of M in the joint frames
M_curly0(:,:)=transpose(Ad_gbar_rel(:,:,n))*[mm(n).*eye(3,3) zeros(3,3);zeros(3,3) Im(n)]*Ad_gbar_rel(:,:,n);
for i=1:n
    M_curly(i,:,:)=transpose(Ad_gcmm_inv(:,:,i))*[mm(i).*eye(3,3) zeros(3,3);zeros(3,3) Im(:,:,i)]*Ad_gcmm_inv(:,:,i);
end
