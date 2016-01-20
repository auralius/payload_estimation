function [ m, c, I, A, res] = estimate_payload(f, n, omega, omegad, a, g)
% Seraina Anne Dual
% 29.07.2014
% Auralis Manurung
% 30.07.2015

% Payload estimation based on IJRR papaer: "Estimation of Inertial 
% Parameters of Manipulator Loads and Links" by Atkenson et. al.
%
% INPUT: 
%   N measured values of forces (f), torques (n), angular velocity
%   (omega), angular acceleration (omegad), linear accleration (a), and
%   gravity vector (g), where:
%     f is N x 3  (N)
%     n is N x 3  (Nmm)
%     omega is N x 3  (rad/s)
%     omegad is N x 3  (rad/s^2)
%     a is N x 3 (m/s^2)
%     g is N x 3 (m/s^2)
%   Measurments are respect to joint origin coordinate system (see p.104
%   of the aforementioned paper).
%   
% OUTPUT: 
%   parameter vector of the inertial parameters:
%   [m, [c2,c2,c3]',[Ixx,Ixy,Ixz,Iyy,Iyz,Izz]']
%   where:
%     m is the mass of the body
%     [c1 c2 c3] are the coordinates of the center of mass of the body
%     [Ixx,Ixy,Ixz,Iyy,Iyz,Izz] are the moments of inertia with respect to 
%     the center of mass of the body. 

%% Prepare some storages
N = length(f(:,1)); % number of measurement points

wn = zeros(N*6,1);
omegax = zeros(N*3,3);
omegadx = zeros(N*3,3);
dotomega = zeros(N*3,6);
dotomegad = zeros(N*3,6);
A = zeros(N*6,10);

%% Build the augmented matrix (see equ. 8)
for i = 1 : N
    wn(6*(i-1)+1:6*i) = [f(i,:),n(i,:)];
    
    % define variables
    omegax(3*(i-1)+1:3*i,:) = skew(omega(i,:)');
    omegadx(3*(i-1)+1:3*i,:) = skew(omegad(i,:)');
    
    g_a = a(i,:)-g(i,:);
    g_ax = skew(-g_a');
    
    dotomega(3*(i-1)+1:3*i,:) = [omega(i,1), omega(i,2), omega(i,3), 0,          0,          0;
                                 0,          omega(i,1), 0,          omega(i,2), omega(i,3), 0;
                                 0,          0,          omega(i,1), 0,          omega(i,2), omega(i,3)];
                             
    dotomegad(3*(i-1)+1:3*i,:)=[omegad(i,1), omegad(i,2), omegad(i,3), 0,           0,           0;
                                0,           omegad(i,1), 0,           omegad(i,2), omegad(i,3), 0;
                                0,           0,           omegad(i,1), 0,           omegad(i,2), omegad(i,3)];
    
    % combine parameter matrix
    A((i-1)*6+1:i*6,:) = [g_a',       (omegadx(3*(i-1)+1:3*i,:)+omegax(3*(i-1)+1:3*i,:)*omegax(3*(i-1)+1:3*i,:)), zeros(3,6);
                          zeros(3,1), g_ax,                                                                       dotomegad(3*(i-1)+1:3*i,:)+omegax(3*(i-1)+1:3*i,:)*dotomega(3*(i-1)+1:3*i,:)];
end

%% Do the least square

p_ls = A\wn;

wn_est = A * p_ls;
res = wn - wn_est;

m_ls = p_ls(1);
cm_ls = p_ls(2:4);

% This is respect to joint origin
I_ls = [p_ls(5), p_ls(6), p_ls(7);
        p_ls(6), p_ls(8), p_ls(9);
        p_ls(7), p_ls(9), p_ls(10)]; 

% Find actual center of gravity and moment of inertia at center of mass
m = m_ls;
c = cm_ls/m_ls;
% Convert to about the center of the body mass (see equ. 6)
I = I_ls-m * ((c.'*c) * eye(3) - (c*c.')); 

% When rank(A) < 10, calculated inertias are worthless, but mass and CoM 
% might still be correct
if rank(A) < 10 
    I = zeros(3,3);
    if rank(A) < 4 % Minimum rank to estimate mass and COM
        m = 0;
        c = zeros(3,1);
    end
end

end