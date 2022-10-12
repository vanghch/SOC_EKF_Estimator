function [Uk_est, Ik_est, SOCk_est] = soc_estimator_ekf(Uk_obs, Ik_obs, SOC_obs, dt)
% 使用EKF估计电池组电量状态 状态方程和观测方程使用一阶戴维宁模型计算，电池参数可配置
% Uk_obs: 端电压观测值
% Ik_obs: 回路电流
% SOC_obs: SOC观测值，可以使用AH-Integrate的计算结果也可以传入EKF上次的估计值
% Uk_est: 极化电压估计值
% Ik_est: 电流估计值
% SOCk_est: SOC估计值

	% 使用Thevnin模型 初始化电池模型内部参数
	global R0 R1 C1 Eta C_N;
	% 协方差矩阵 过程噪声以及观测噪声矩阵
	global P Q R Wk Vk;
	% 迭代中的各种状态变量
	global Uk Ik SOCk;
	
	% 状态方程 可以根据状态转移是否为线性或者非线性选择使用f(xk, uk)的形式表示或者用矩阵形式表示
	Uk_ = exp(-dt/(R1*C1)) * Uk + R1 * (1 - exp(-dt/(R1*C1))) * Ik;	% 这里先验估计的Uk_为极化电压
	SOCk_ = SOCk + (Eta*dt/C_N) * Ik;								% 这里先验估计的SOCk为系统SOC状态
	Xk_ = [Uk_; SOCk_];												% 先验估计状态
	Ak = [exp(-dt/(R1*C1)), 0; 0, 1];								% Ak为状态方程两个公式所对应的雅可比矩阵

	% 计算误差协方差
	P_ = Ak*P*Ak' + Wk*Q*Wk';										% 先验估计计算出的误差协方差矩阵

	% 观测方程 EMF与SOC的5阶拟合系数矩阵为[1.6026 -8.2896 14.8878 -12.0873 5.5781 2.5047]
	% TODO:在观测方程中R0也是SOC的函数，此处用一个恒定的值代替，后续待补充
	Uk_pola = Uk_obs - R0*Ik_obs - (1.6026*(SOC_obs^5) - 8.2896*(SOC_obs^4) + 14.8878*(SOC_obs^3) - 12.0873*(SOC_obs^2) + 5.5781*SOC_obs + 2.5047);	% 这里的Uk_pola为计算出的极化电压
	Xk_obs = [Uk_pola; SOC_obs];									% 观测状态
	Ck = [1, (8.013*SOCk^4) - (33.1584*SOCk^3) + (44.6634*SOCk^2) - (24.1746*SOCk) + 5.5781];	% Ck为两个观测方程对应的雅可比矩阵

	% 计算卡尔曼增益系数
	Kk = P_ * Ck' / (Ck*P_*Ck' + Vk'*R*Vk);							% 更新卡尔曼系数

	% 计算卡尔曼滤波输出
	Xk_est = Xk_ + Kk.*(Xk_obs - Xk_);								% 后验更新估计状态

	% 更新误差协方差矩阵
	P = (eye(2) - Kk*Ck) * P_;										% 后验更新误差协方差矩阵
	
	Uk_est = Xk_est(1, 1);
	Ik_est = Ik_obs;
	SOCk_est = Xk_est(2, 1);
	
end