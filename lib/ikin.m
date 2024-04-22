function q = ikin(S, M, q0, V)

    currentQ = q0;
    count = 0;
    currentPose = MatrixLog6(fkine(S,M,currentQ, 'space'));
    currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';
    targetPose = V;

    while norm(targetPose - currentPose) > 1e-6 && count < 5000
        J = jacob0(S, currentQ);

        lambda = 0.5;
        Jstar = J'/(J*J' + lambda^2*eye(6));

        % Calculate the Newton-Raphson method Jacobian
        deltaQ = Jstar*(targetPose - currentPose);
        deltaQ = max(min(deltaQ, 0.3), -0.3);

        currentQ = currentQ + deltaQ';

        T = fkine(S,M,currentQ, 'space');
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

        count = count + 1;
    end
    
    q = currentQ;
end