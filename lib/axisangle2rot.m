function R = axisangle2rot(omega,theta)
    ssm = [0 -omega(3) omega(2);
           omega(3) 0 -omega(1);
           -omega(2) omega(1) 0];
    R = eye(3) + sin(theta)*ssm + (1-cos(theta))*ssm^2;
end