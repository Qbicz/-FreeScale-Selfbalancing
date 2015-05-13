%% Nieliniowy model wahad³a odwróconego
% z wykorzystaniem Simulinka i Simscape
clear all; close all; clc;

% parametry wahadla
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

% regulator
Kp = 100;
Ki = 1;
Kd = 20;

Kpx = 5;
Kdx = 10;

amplit = 1000; % amplituda zaklocenia
width = 0.001; % czas zaklocenia
Tsim = 10; % czas symulacji
sim('Pend_Simscape_PID');

%% Wykresy
figure;
subplot(2,1,1);
plot(th_out);
title('Wahad³o odwrócone');
ylabel('Wychylenie wahadla'); xlabel('')
subplot(2,1,2);
plot(x_out); title('');
ylabel('Po³o¿enie kó³'); xlabel('Czas [s]');

