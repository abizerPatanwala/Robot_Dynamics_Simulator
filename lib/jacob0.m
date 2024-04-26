function J = jacob0(S,q) 
    T = zeros(4, 4, size(q,2));
    for i = 1:size(q,2)
        T(:,:,i) = twist2ht(S(:,i), q(i));
    end
    J = zeros(6, size(q,2));
    curr_trans = eye(4);
    for i = 1:size(q,2)
        J(:,i) = adjoint(S(:,i),curr_trans);
        curr_trans = curr_trans * T(:,:,i); 
    end
end