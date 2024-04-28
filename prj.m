%% ******************************* ECE 864 Project ********************************
% Student ID : 181248852
% Student full name: Sayed Hamed Khatounabadi
%% --------------------- Question 3- part A ---------------------------------------
clear;clc;close all;
numb = 100;
r_x_origin = 1e-5;
r_y_origin = 1e-5;
n = linspace(0,100,100);
r_x_ideal = zeros(100,1);
r_y_ideal = zeros(100,1);
for i= 1:1:100
    r_x_ideal(i,1) = r_x_origin - 0.2*n(1,i);
    r_y_ideal(i,1) = r_y_origin + 0.2*n(1,i);
end
delta = 1;
A = diag([1,1,1,1]);
A(1,3) = delta;
A(2,4) = delta;
var_driving = 1e-4;

mu = [0; 0; 0; 0];
Q = [0 0 0 0; 0, 0, 0, 0; 0, 0, var_driving, 0; 0, 0, 0, var_driving];
rng('default')  % For reproducibility
u = mvnrnd(mu,Q,numb)';
s = zeros(4, numb+1);
s(:,1) = [r_x_origin; r_y_origin; -0.2; 0.2];
for i= 1:1:numb
    s(:,i+1) = A * s(:,i) + u(:,i);
end

figure;
plot(r_x_ideal, r_y_ideal, 'Color', 'green', 'LineWidth',2);
hold on;
plot(s(1, :), s(2, :), 'Color', 'blue', 'LineWidth',2); 
hold on;
plot(0, 0, 'o', 'Color', 'red', 'LineWidth',2);
title(["Vehicle tracking for input variance", num2str(var_driving)]); ylabel("r_y"); xlabel("r_x"); grid on;
xlim([-20 0]); ylim([0 20])
legend("Ideal track","True track", "Origin");
%% --------------------- Question 3- part B ---------------------------------------
x = zeros(2, numb);
h_sn = zeros(2, numb);
angle = zeros(1, numb);
% sd_r = 0.05;
% sd_beta =0.05;
sd_r = 0.3162;
sd_beta = 0.1;
mu = [0; 0];
C = [sd_r^2, 0; 0 , sd_beta^2];
rng('default')  % For reproducibility
w = mvnrnd(mu,C,numb)';

for i= 1:1:numb
    if s(1,i) <=0
        angle(1,i) = 180*(1/pi)*(pi + atan(s(2,i)/s(1,i)));
    else
        angle(1,i) = 180*(1/pi)*atan(s(2,i)/s(1,i));
    end
    h_sn(:,i) = [sqrt(s(1,i).^2 + s(2,i).^2); pi*(1/180)*angle(1,i)];
    x(:,i) = h_sn(:,i) + w(:,i);
end

r_x_measured = x(1,:).*cos(x(2,:));
r_y_measured = x(1,:).*sin(x(2,:));

figure;
plot(r_x_measured, r_y_measured, 'Color', 'red', 'LineWidth',2); 
hold on;
plot(s(1, :), s(2, :), 'Color', 'blue', 'LineWidth',2); 
hold on;
plot(0, 0, 'o', 'Color', 'red', 'LineWidth',2);
title(sprintf('Vehicle tracking\ninput variance %.4f range varince: %.4f bearing varince:%.4f', ...
    var_driving, sd_r^2, sd_beta^2))
ylabel("r_y"); xlabel("r_x"); grid on;
% xlim([-9 0]); ylim([0 15.5])
legend("Measured track", "True track", "Origin");

%% --------------------- Question 3- part C ---------------------------------------

s_hat_init = [r_x_origin ; r_y_origin; 0; 0];
s_hat_n_given_n_1 = zeros(4, numb);
s_hat_n_given_n = zeros(4, numb+1);
s_hat_n_given_n(:,1) = s_hat_init;

M_init = 100* diag([1,1,1,1]);
M_n_given_n_1 = zeros(4, 4, numb);
M_n_given_n = zeros(4, 4, numb+1);
M_n_given_n(:, :, 1) = M_init;

k_n = zeros(4, 2, numb);
for i= 2:1:numb+1
    s_hat_n_given_n_1(:,i-1) = A * s_hat_n_given_n(:,i-1);
    M_n_given_n_1(:, :, i-1) = A * M_n_given_n(:, :, i-1) * A' + Q;
    k_n(:, :, i-1) = M_n_given_n_1(:, :, i-1) * my_H_n(s_hat_n_given_n_1(:,i-1))' * ...
        (C + my_H_n(s_hat_n_given_n_1(:,i-1)) * M_n_given_n_1(:, :, i-1) * my_H_n(s_hat_n_given_n_1(:,i-1))')^(-1);
    s_hat_n_given_n(:,i) = s_hat_n_given_n_1(:,i-1) + k_n(:, :, i-1) * (x(:,i-1) - my_h_sn(s_hat_n_given_n_1(:,i-1)));
    M_n_given_n(:, :, i) = (eye(4) - k_n(:, :, i-1) * my_H_n(s_hat_n_given_n_1(:,i-1))) * ...
        M_n_given_n_1(:, :, i-1);
end

figure;
plot(s_hat_n_given_n(1, :), s_hat_n_given_n(2, :), 'Color', 'black', 'LineWidth', 2);
hold on;
plot(s(1, :), s(2, :), 'Color', 'blue', 'LineWidth', 2); 
hold on;
plot(0, 0, 'o', 'Color', 'red', 'LineWidth',2);
title(sprintf('Vehicle tracking\ninput variance %.4f range varince: %.4f bearing varince:%.4f', ...
    var_driving, sd_r^2, sd_beta^2))
ylabel("r_y"); xlabel("r_x"); grid on;
legend("Kalman filter estimated track", "True track", "Origin");
%% --------------------- Question 3- part D ---------------------------------------
figure;
plot(s_hat_n_given_n(1, :), s_hat_n_given_n(2, :), 'Color', 'black', 'LineWidth',1.5);
hold on;
plot(r_x_measured, r_y_measured, 'Color', 'red', 'LineWidth',1.5); 
hold on;
plot(s(1, :), s(2, :), 'Color', 'blue', 'LineWidth',1.5);
hold on;
plot(0, 0, 'o', 'Color', 'red', 'LineWidth',2);
title(sprintf('Vehicle tracking\ninput variance %.4f range varince: %.4f bearing varince:%.4f', ...
    var_driving, sd_r^2, sd_beta^2))
ylabel("r_y"); xlabel("r_x"); grid on;
legend("Kalman filter estimated track", "Measured track", "True track", "Origin");

%% --------------------- Question 3- online Kalman Filter --------------------------
figure; hold all;
ylim([-1,16])
xlim([-11,1])
n=1;
plot(s_hat_n_given_n(1, n), s_hat_n_given_n(2, n),'*-', 'Color', 'black', 'LineWidth',1, 'DisplayName','Kalman filter estimated track');
    plot(r_x_measured(1,n), r_y_measured(1,n), '*-', 'Color', 'red', 'LineWidth',1,'DisplayName','Measured Track'); 
    plot(s(1, n), s(2, n), '*-', 'Color', 'blue', 'LineWidth',1,'DisplayName','True Track'); 
 legend('show')
for n=2:1:numb
    plot(s_hat_n_given_n(1, n), s_hat_n_given_n(2, n),'*-', 'Color', 'black', 'LineWidth',1);
    plot(r_x_measured(1,n), r_y_measured(1,n), '*-', 'Color', 'red', 'LineWidth',1); 
    plot(s(1, n), s(2, n), '*-', 'Color', 'blue', 'LineWidth',1); 
    legend('off', 'off', 'off')
    if n>1
        plot(s_hat_n_given_n(1, n-1:n), s_hat_n_given_n(2, n-1:n),'-', 'Color', 'black', 'LineWidth',1);
        plot(r_x_measured(1,n-1:n), r_y_measured(1,n-1:n), '-', 'Color', 'red', 'LineWidth',1); 
        plot(s(1, n-1:n), s(2, n-1:n), '-', 'Color', 'blue', 'LineWidth',1);
        legend("Kalman filter estimated track", "Measured track", "True track");
    end
    pause(0.3);
    title("Vehicle tracking"); ylabel("r_y"); xlabel("r_x"); grid on;
    
end