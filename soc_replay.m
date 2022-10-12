csv_file = readmatrix('WAVE(2022.3.31-20.33.57).csv');

battery_tim = csv_file(:,6);
battery_soc = csv_file(:,8);
battery_vol = csv_file(:,10);
battery_cur = csv_file(:,4);
battery_soc_uoc = csv_file(:, 7);

% 使用Thevnin模型 初始化电池模型内部参数
global R0 R1 C1 Eta C_N;
R0 = 0.0096;
R1 = 0.0071;
C1 = 50;
Eta = 0.97;
C_N = 3*3600;
dt = 0.1;
len = length(battery_tim);
accumlate = 0.0;

% 定义数据用于记录每次迭代状态
Uoc = nan(len, 1);
SOC = nan(len, 1);
U = nan(len, 1);
SOC_AH = nan(len, 1);
I = nan(len, 1);

% 初始化EKF迭代时的初始状态
global Uk SOCk Ik;
Uk = battery_vol(1);
SOCk = battery_soc(1);
Ik = battery_cur(1);

% 定义过程噪声与观测噪声矩阵
global P Q R Wk Vk;
P = [1e-8 0.0; 0.0 1e-6];
Q = 0.001;
R = 1000;
Wk = [0.001; 0.001];
Vk = [1000; 1000];

Uoc(1) = Uk;
SOC(1) = SOCk;
SOC_AH(1) = SOCk;
I(1) = Ik;
U(1) = 0;

for i = 2 : len
	Uk_obs = battery_vol(i);
	SOCk_obs =  SOCk;
	Ik_obs = -battery_cur(i);
	
	dt = battery_tim(i) - battery_tim(i-1);
	if dt < 0.001
		dt = 0.001;
	end
	
	accumlate = accumlate + Ik_obs * dt;
	
	[Uk, Ik, SOCk] = soc_estimator_ekf(Uk_obs, Ik_obs, SOCk_obs, dt);

	Uoc(i) = Uk + polyval([1.6026 -8.2896 14.8878 -12.0873 5.5781 2.5047], SOCk) + Ik * R0;
	SOC(i) = SOCk;
	SOC_AH(i) = battery_soc(1) + (accumlate / C_N);
	U(i) = Uk;
	I(i) = -Ik_obs;
end

figure(1);
plot(battery_vol, 'r.-', 'LineWidth', 2); hold on;
plot(Uoc, 'g.-', 'LineWidth', 2); hold on;
plot(U, 'b.-', 'LineWidth', 2); hold on;
legend('电池端电压观测', '电池端电压估计', '极化电压估计');
xlabel('T*10s'); ylabel('电压/v');
grid on;
figure(2);
plot(I, 'c.-', 'LineWidth', 2); hold on;
plot(SOC_AH.*10, 'r.-', 'LineWidth', 2); hold on;
plot(SOC.*10, 'g.-',  'LineWidth', 2); hold on;
plot(battery_soc_uoc.*10, 'b.-', 'LineWidth', 2); hold on;
legend('电流', 'AH积分结果', 'EKF估计结果', '回路电压计算SOC');
xlabel('T'); ylabel('SOC*10');
grid on;