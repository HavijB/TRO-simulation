function [Ad_frak] = Ad_frak(i,j,Xi_m,q_m)

if i>=j
    g_temp=eye(4);
    for k=i:-1:j
        g_temp=g_temp*expm(-Xi_m(:,:,i).*q_m(i));
    end
    Ad_frak=Adjoint(g_temp);
end

end