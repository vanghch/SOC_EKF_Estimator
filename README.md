- 文件说明：

    1. WAVE(2022.3.31-20.33.57).csv文件用于测试的demo数据
	1. soc_replay.m是主文件，直接运行该文件即可得到文件夹中示例的结果
	1. soc_estimator_ekf.m是估计部分，使用了一阶戴维宁模型来估计极化电压和极化电容
	
- 估计原理：
    1. 电路模型<img width="218" alt="Image" src="https://user-images.githubusercontent.com/37259952/195364405-644ed0f0-d565-4040-8bdd-5f1187ad145b.png">
    1. 估计原理：<img width="738" alt="86E8C322-A859-41f1-83B5-17385467E682" src="https://user-images.githubusercontent.com/37259952/195364494-4308675c-ad6c-458b-893d-3a2b080b5404.png">
