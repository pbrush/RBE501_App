function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
% YOUR CODE HERE 
S = params.S;
g = params.g;
M = params.M;
G = params.G;
jointVel = params.jointVel;
jointPos = params.jointPos;
jointAcc = params.jointAcc;
Ftip = params.Ftip;


V = zeros(6, size(S,2)+1);
Vdot = zeros(6, size(S,2)+1);
tau = zeros(size(S,2), 1);
A = zeros(6, size(S,2));

V(:, 1) = [0 0 0 0 0 0]';
Vdot(4:6, 1) = -g;

M_joint = zeros(size(M));
M_joint(:, :, 1) = M(:, :, 1);
for i = 2:size(M,3)
    M_joint(:, :, i) = M_joint(:,:,i-1) * M(:, :, i);
end

T = zeros(size(M));
for i = 1:size(S,2)
    A(:, i) = adjoint(inv(M_joint(:,:,i))) * S(:,i);
    T(:, :, i) = twist2ht(-A(:, i), jointPos(i)) * inv(M(:, :, i));
end
T(:, :, end) = inv(M(:, :, end));

for i = 1:size(S,2)
    V(:,i+1) = A(:,i) * jointVel(i) + adjoint(T(:, :, i)) * V(:, i);
    Vdot(:, i+1) = A(:,i) * jointAcc(i) + adjoint(T(:, :, i)) * Vdot(:, i) + ad(V(:, i+1)) * A(:,i) * jointVel(i);
end

W = zeros(6, size(S,2)+1);
W(:, size(S,2)+1) = Ftip;
% Backward iterations
% YOUR CODE HERE
for i = size(S,2):-1:1
    W(:, i) = G(:, :, i)*Vdot(:,i+1) - ad(V(:, i+1))'*G(:, :, i)*V(:, i+1) + adjoint(T(:, :, i+1))'*W(:, i+1);    
    tau(i) = W(:, i)'*A(:,i);
end


function AdT = adjoint(T)
        % your code here
    p = T(1:3, 4);
    R = T(1:3, 1:3);
    ssm = [0 -p(3) p(2);
           p(3) 0 -p(1);
           -p(2) p(1) 0];
    
    AdT = [R zeros(3); ssm*R R];
    
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
    
function T = twist2ht(S,theta)
    omega = S(1:3);
    v = S(4:6);
    ssm = [0 -omega(3) omega(2);
           omega(3) 0 -omega(1);
           -omega(2) omega(1) 0];
    R = axisangle2rot(omega,theta);
    T = [R (eye(3)*theta + (1-cos(theta))*ssm + (theta - sin(theta))*ssm^2)*v;
        0 0 0 1];

function R = axisangle2rot(omega,theta)
    ssm = [0 -omega(3) omega(2);
           omega(3) 0 -omega(1);
           -omega(2) omega(1) 0];
    R = eye(3) + sin(theta)*ssm + (1-cos(theta))*ssm^2;