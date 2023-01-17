function [adjoint_low] = adjoint_low(V)
% returns the adjoint of the given transformation
    
    % Extract linear and Angular portion
    v=V(1:3);
    w=V(4:6);
    % Form matrix elements
    w_tilde=skewsym(w);
    v_tilde=skewsym(v);
    adjoint_low=[w_tilde v_tilde;zeros(3) w_tilde];
end