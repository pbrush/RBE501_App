function J_b = jacobe(S,M,q)    
    J_s = jacob0(S,q);
    J_b = zeros(6, size(S, 2));
    
    T = fkine(S,M,q, 'space');

    for i = 1:size(S, 2)
        J_b(1:6, i) = twistspace2body(J_s(:, i), inv(T));
    end
end