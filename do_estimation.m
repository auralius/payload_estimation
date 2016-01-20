k = 200; % Starting index

% Force, torque and gravity are respect to global/world frame
% Therefore, we need the rotation matrix of the rigid body from body to 
% global frame
rot = R(k:end, :);       % from body to global frame
f_g = force(k:end, :)';  % forces in global frame [N]
n_g = torque(k:end, :)'; % torques in global frame [Nm]
g_g = [0, 0, -9.81]';    % global frame [m/s^2]

omega = omega(k:end, :);          % angular velocity in body frame [rad/s]
omegad_dot = omega_dot(k:end, :); % angular acceleration in body frame [rad/s^s]
a = acc(k:end, :);                % joint acceleration in body frame [m/s^s]

% Very crucial: convert forces, torques, and gravity vector into body frame
N = length(f_g(1, :));
f = zeros(N, 3);
n = zeros(N, 3);
g = zeros(N, 3);

for i = 1 : N
    roti = reshape(rot(i,:), 3, 3);
    f(i,:) = (roti.' * f_g(:,i))'; 
    n(i,:) = (roti.' * n_g(:,i))';
    g(i,:) = roti.' * g_g;
end

% Run the estimation:
[m, c, I, A, res] = estimate_payload(f, n, omega, omegad_dot, a, g);

disp('Mass:')
disp(m)
disp('CoM:')
disp(c)
disp('Inertias:')
disp(I)
