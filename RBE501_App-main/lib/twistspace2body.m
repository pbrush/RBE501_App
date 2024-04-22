function V_b = twistspace2body(V_s,T)
    V_b = adjoint(V_s, T);
end