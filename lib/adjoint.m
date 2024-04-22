function AdT = adjoint(T)
        % your code here
    p = T(1:3, 4);
    R = T(1:3, 1:3);
    ssm = [0 -p(3) p(2);
           p(3) 0 -p(1);
           -p(2) p(1) 0];
    
    AdT = [R zeros(3); ssm*R R];
end