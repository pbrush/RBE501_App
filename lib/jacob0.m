function J = jacob0(S,q)
   
    J = zeros(6, width(S));
    for i = 1:width(S)
        T = eye(4);
        % Fill each column of the jacobian with the adjoint of the prior
        % screw vectors.
        for j = 1:i
            T = T * twist2ht(S(1:6, j), q(j));
        end
        J(1:6, i) = adjoint(T) * S(1:6, i);
    end
    
end