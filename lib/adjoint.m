function twist_inB = adjoint(twist_inA,T_AB)
    R = T_AB(1:3,1:3);
    p = T_AB(1:3,4);
    skew_p = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    Adj_T = [R zeros(3,3); skew_p*R R];
    twist_inB = Adj_T*twist_inA; 
end

