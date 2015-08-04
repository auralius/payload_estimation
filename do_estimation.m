k = 200; % Starting index

rot = R(k:end, :);       % from local to global
f_g = force(k:end, :)';  % forces in global coordinate system [N]
n_g = torque(k:end, :)'; % torques in global coordinate system [Nm]
g_g = [0, 0, -9.81]';    % m/s^2

omega = omega(k:end, :);          % angular velocity in body coordinate system [rad/s]
omegad_dot = omega_dot(k:end, :); % angular acceleration in body coordinate system [rad/s^s]
a = acc(k:end, :);                % joint acceleration in body coordinate system [m/s^s]

% Very crucial: convert forces and torques into body coordinate system
N = length(f_g(1,:));
f = zeros(N,3);
n = zeros(N,3);
g = zeros(N,3);

for i = 1 : N
    roti=reshape(rot(i,:),3,3);
    f(i,:)=(roti.'*f_g(:,i))'; %Nx3
    n(i,:)=(roti.'*n_g(:,i))'; %Nx3
    g(i,:)=roti.'*g_g; %gravitational acceleration [m/s^2]
end

% Run the estimation: 
[m, c, I, A, res] = estimate_payload(f, n, omega, omegad_dot, a, g);

disp('Mass:')
disp(m)
disp('CoM:')
disp(c)
disp('Inertias:')
disp(I)
