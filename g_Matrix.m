function [g] = g_Matrix(qm,b,xi) %#codegen
% compute the tranformation matrix containing the rotation matrix R and
% the translation vector s between two joints

%Revolute joint
theta=qm;
  
% Form the g Matrix
g=expm(xi*theta)*inv([eye(3),b;zeros(1,3),1]);
% g=inv([eye(3),b;zeros(1,3),1])*[R,s;zeros(1,3),1];

end