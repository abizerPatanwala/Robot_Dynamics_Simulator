function waypoints = ikin(S, M, path)
n = size(S,2);
nPts = size(path,2);
waypoints = zeros(n,nPts);
currentQ = zeros(1, n);
lambda = 0.5;
for i=1:nPts
    T = fkine(S, M, currentQ,'space');
    currentPose = T(1:3, 4);
    while norm(path(:,i) - currentPose) > 1e-3 
        Ja = jacoba(S, M, currentQ);
        deltaQ = Ja' * pinv(Ja*Ja' + lambda^2 * eye(3)) * (path(:,i) - currentPose);
        currentQ = currentQ + deltaQ'; 
        T = fkine(S, M, currentQ,'space');
        currentPose = T(1:3, 4); 
    end
    waypoints(:,i) = currentQ; 
end
end