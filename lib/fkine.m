function T = fkine(S,M,q,frame)
    T = eye(4);
    for i = 1:size(S,2)
        T = T * twist2ht(S(:,i),q(i));
    end
    if strcmp(frame, 'space')
        T = T * M;
    end
    if strcmp(frame, 'body')
        T = M * T;
    end
end
