function J_a = jacoba(S,M,q)    
    T = fkine(S,M,q, 'space');
    R = T(1:3,1:3);
    J_s = jacob0(S,q);
    P = T(1:3,4);
    p = [0 -P(3) P(2) ; P(3) 0 -P(1) ; -P(2) P(1) 0 ];
    adj = [R zeros(3,3); p*R R];
    J_b = inv(adj)*J_s;
    J_vb = J_b(4:6,:);
    J_a = R*J_vb;
end