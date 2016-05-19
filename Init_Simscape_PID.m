%% Nieliniowy model wahad³a odwróconego
% z wykorzystaniem Simulinka i Simscape
clear all; close all; clc;

% parametry wahadla
M = 0.4;        % masa wozka
m = 0.5;        % masa wahadla
b = 0.1;        % tarcie
I = m*l*l/3;    % bezwladnosc preta (0.006)
g = 9.8;    
l = 0.2;        % odleglosc od srodka ciezkosci

% regulator
Kp = 80;
Ki = 1;
Kd = 20;

Kpx = 5; % 5
Kdx = 10; % 10

amplit = 1000; % amplituda zaklocenia
width = 0.001; % czas zaklocenia
Tsim = 5; % czas symulacji
a = 0.04; % rozmiar kol (promien)
szer = 0.03; % szerokosc pojazdu (polowa)
sim('Pend_Simscape_PID');

%% Wykresy
figure;
subplot(2,1,1);
plot(th_out);
title('Wahad³o odwrócone');
ylabel('Wychylenie wahadla'); xlabel('')
subplot(2,1,2);
plot(x_out); title(sprintf('Kpx = %d, Kdx = %d', Kpx, Kdx));
ylabel('Po³o¿enie kó³'); xlabel('Czas [s]');

