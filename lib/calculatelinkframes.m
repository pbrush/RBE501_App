function [M01, M12, M23, M34, M45, M56, M67, M78] = calculatelinkframes(ur5)
  Mj1 = tdh(0, ur5.d(1), ur5.a(1), ur5.alpha(1));
  Mj2 = Mj1 * tdh(0, ur5.d(2), ur5.a(2), ur5.alpha(2));
  Mj3 = Mj2 * tdh(0, ur5.d(3), ur5.a(3), ur5.alpha(3));
  Mj4 = Mj3 * tdh(0, ur5.d(4), ur5.a(4), ur5.alpha(4));
  Mj5 = Mj4 * tdh(0, ur5.d(5), ur5.a(5), ur5.alpha(5));
  Mj6 = Mj5 * tdh(0, ur5.d(6), ur5.a(6), ur5.alpha(6));
  Mj7 = Mj6 * tdh(0, ur5.d(7), ur5.a(7), ur5.alpha(7));

  M1 = Mj1 * [eye(3) [0, -0.04, -0.05]'; 0 0 0 1];
  M2 = Mj2 * [eye(3) [0, -0.04, 0.06]'; 0 0 0 1];
  M3 = Mj3 * [eye(3) [0.01, 0.01, -0.05]'; 0 0 0 1];
  M4 = Mj4 * [eye(3) [-0.03, 0.03, 0.02]'; 0 0 0 1];
  M5 = Mj5 * [eye(3) [0, 0.04, -0.12]'; 0 0 0 1];
  M6 = Mj6 * [eye(3) [0.04, 0, 0]'; 0 0 0 1];
  M7 = Mj7 * [eye(3) [0, 0, 0.08]'; 0 0 0 1];
  
  M01 = M1;
  M12 = pinv(pinv(M2)*M1);
  M23 = pinv(pinv(M3)*M2);
  M34 = pinv(pinv(M4)*M3);
  M45 = pinv(pinv(M5)*M4);
  M56 = pinv(pinv(M6)*M5);
  M67 = pinv(pinv(M7)*M6);
  M78 = eye(4);
end

