function [ad_frak] = Ad_frak(i,j,Xi_m,q_m)

if i>=j
    g_temp=eye(4);
    for k=i:-1:j
        g_temp=g_temp*g_Matrix(Xi_m(:,k),-q_m(k));
    end
    ad_frak=Adjoint(g_temp);
end

end