function adV = ad(V)
    omega = V(1:3);
    v = V(4:6);
    
    adV = [skew(omega) zeros(3); skew(v) skew(omega)];
end