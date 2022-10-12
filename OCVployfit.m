p = polyfit(Ocv_data(1,:), Ocv_data(2,:), 5);
p1 = polyfit(Ocv_data(2,:), Ocv_data(1,:), 5);

figure(1);
Soc = (0 : 0.01 : 1);
Ocv = polyval(p, Soc);
plot(Ocv, Soc, Ocv_data(2,:), Ocv_data(1,:), 'o');
grid on;
xlabel('OCV/V'); ylabel('SOC*100%');

figure(2);
Ocv1 = (2.4 : 0.01 : 4.2);
Soc1 = polyval(p1, Ocv);
plot(Soc, Ocv, Ocv_data(1,:), Ocv_data(2,:), 'o');
grid on;
xlabel('SOC*100%'); ylabel('OCV/V');
