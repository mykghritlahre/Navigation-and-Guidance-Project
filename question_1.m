
A = [
    -1.2185034e-4  -1.9939147e-4  0          -3.217405e1 ;
    -4.4309215e-4  -2.2763856e-3  2.500353e2    0          ;
     2.5579606e-7  -3.8243728e-3  1.894566e-2   0          ;
     0            0            1           0          
];
B = [0; -0.14402528; -0.07354475; 0];
C = eye(4);
D = zeros(4, 1);
sys = ss(A, B, C, D);

%%eigen values and lqr
eig_A = eig(A);
Qbar = C' * C;
R = 1;
[K, S, e] = lqr(A, B, Qbar, R);
eig_A_minus_BK = eig(A - B * K);

% discrete
Ts = 0.01;
Acl = A - B * K;
B_placeholder = zeros(size(A, 1), 1);
D_placeholder = zeros(size(C, 1), 1);

while true
    sys_cl = ss(Acl, B_placeholder, C, D_placeholder);
    sys_cl_discrete = c2d(sys_cl, Ts, 'zoh');
    F = sys_cl_discrete.A;
    eig_F = eig(F);
    if all(abs(eig_F) < 1)
        break;
    else
        Ts = Ts + 0.01;
    end
end

% Generate true states and measurements with noise
alpha = 1;
rng(alpha);
process_noise_std = 0.002;
measurement_noise_std = 0.002;
Qbar = (process_noise_std * eye(size(F))) .^ 2;
R = (measurement_noise_std * eye(size(F))) .^ 2;
N = 100;
x_true = zeros(size(F, 1), N);
y_true = zeros(size(F, 1), N);
w_k = zeros(size(F, 1), N);
v_k = zeros(size(F, 1), N);
x_true(:, 1) = [0; 0; 0; 0];
y_true(:, 1) = [0; 0; 0; 0];

for k = 1:N-1
    w_k(:, k) = process_noise_std * randn(size(F, 1), 1);
    v_k(:, k) = measurement_noise_std * randn(size(F, 1), 1);
    x_true(:, k+1) = F * x_true(:, k) + w_k(:, k);
    y_true(:, k) = x_true(:, k) + v_k(:, k);
end

figure;
for i = 1:size(x_true, 1)
    subplot(size(x_true, 1), 1, i);
    plot(1:N, x_true(i, :), 'b', 'LineWidth', 1.5);
    xlabel('Time Step');
    ylabel(['State ', num2str(i)]);
    title(['True State ', num2str(i)]);
end

% kalman filter
x_est = zeros(size(F, 1), N);
x_pred = zeros(size(F, 1), N);
P = eye(size(F));
I = eye(size(F));

for k = 1:N-1
    x_pred(:, k + 1) = F * x_est(:, k);
    P_pred = F * P * F' + Qbar;
    K = P_pred * inv(P_pred + R);
    x_est(:, k + 1) = x_pred(:, k + 1) + K * (y_true(:, k) - x_pred(:, k + 1));
    P = (I - K) * P_pred;
end

figure;
for i = 1:size(x_true, 1)
    subplot(size(x_true, 1), 1, i);
    plot(1:N, x_true(i, :), 'b', 'LineWidth', 1.5);
    hold on;
    plot(1:N, x_est(i, :), 'r--', 'LineWidth', 1.5);
    xlabel('Time Step');
    ylabel(['State ', num2str(i)]);
    legend('True State', 'Estimated State');
    title(['True and Estimated State ', num2str(i)]);
end
