function T = fkine(S,M,q,frame)
    
    if strcmp('body', frame)
        T = M;
        for i = 1:size(S, 2)
            T = T*twist2ht(S(:,i), q(i));
        end
    else
        T = eye;
        for i = 1:size(S, 2)
            T = T*twist2ht(S(:,i), q(i));
        end
        T = T*M;
    end

end