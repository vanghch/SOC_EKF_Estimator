t = 0 : 0.1 : 2*pi;
dt = 0.1;
len = length(t);

noise1 = 10 * rand(1, len);
noise2 = 2 * rand(1, len);

X = 10 + 5 .* t + 0.1 .* t .* t;
V = 2 + 1 .* t;
A = 0.2;
Xob = X + noise1;
Vob = V + noise2;
Xest = nan(len, 1);
Vest = nan(len, 1);

Xest(1) = X(1);
Vest(1) = V(1);

F = [1 dt; 0 1];
B = [0.5*dt*dt; dt];
H = [1 0; 0 1];
P = [0.1 0.1; 0.1 0.1];
Q = [0.01 0.01; 0.01 0.01];
R = [100 100; 100 100];

Xk = [X(1); V(1)];

for i = 2 : len
	a = A;
	zk = [Xob(i); Vob(i)];
	Xk_ = F * Xk + B .* a;
	P_ = F * P * F' + Q;
	K = (P_ * H') ./ (H * P_ * H' + R);
	Xk = Xk_ + K * (zk - H * Xk_);
	P = (eye(2) - K * H) * P_;
	
	Xest(i) = Xk(1);
	Vest(i) = Xk(2);
end

figure(1);
plot(X, 'r', 'LineWidth', 2); hold on;
plot(Xob, 'g', 'LineWidth', 2); hold on;
plot(Xest, 'b', 'LineWidth', 2); hold on;
legend('标准', '观测', '估计');

figure(2);
plot(V, 'r', 'LineWidth', 2); hold on;
plot(Vob, 'g', 'LineWidth', 2); hold on;
plot(Vest, 'b', 'LineWidth', 2); hold on;
legend('标准', '观测', '估计');