%% Zlinearyzowany model wahad³a odwróconego
% z http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
% dokladnie te rownania co Tomera z wydzialu EiA, PG

clear all; close all; clc;

%% parametry

M = 0.5;        % masa wozka
m = 0.2;        % masa wahadla
b = 0.1;        % wspolczynnik tarcia ruchu wozka
I = 0.006;      % moment bezwladnosci wahadla
g = 9.8;        % przyspieszenie ziemskie
l = 0.3;        % odleglosc do srodka masy wahadla

%% Reprezentacja TF
q = (M+m)*(I+m*l^2) - (m*l)^2;
s = tf('s');

% transmitancja polozenia wozka X(s)/U(s)
G_x = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 ...
    + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

% transmitancja kata odchylenia wahadla od dolnego punktu rownowagi
G_phi = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [G_x; G_phi];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf, 'InputName', inputs)
set(sys_tf, 'OutputName', outputs)

sys_tf

%% Reprezentacja SS
% 
% p = I*(M+m)+M*m*l^2; % mianownik macierzy A i B
% 
% A = [0      1              0           0;
%      0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
%      0      0              0           1;
%      0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
% B = [     0;
%      (I+m*l^2)/p;
%           0;
%         m*l/p];
% C = [1 0 0 0;
%      0 0 1 0];
% D = [0;
%      0];
% 
% states = {'x', 'x_dot', 'phi', 'phi_dot'};
% inputs = {'u'};
% outputs = {'x'; 'phi'};
%  
% sys_ss = ss(A,B,C,D, 'statename', states, ...
%     'inputname', inputs, 'outputname', outputs);
% 
% %% Porownanie
% 
% sys_tf_2 = tf(sys_ss)

% daje te same transmitancje, przeskalowane przez 4.3472e+05
% 
% sys_tf_2 =
%  
%   From input "u" to output...
%                  1.818 s^2 - 44.55
%    x:  --------------------------------------
%        s^4 + 0.1818 s^3 - 31.18 s^2 - 4.455 s
%  
%                 4.545 s + 2.539e-16
%    phi:  ----------------------------------
%          s^3 + 0.1818 s^2 - 31.18 s - 4.455
%
%% Obserwowalnosc
% 
% ob = obsv(sys_ss);
% observability = rank(ob)

%% Odpowiedz impulsowa

t=0:0.02:1;
figure;
impulse(sys_tf,t);
title('OdpowiedŸ impulsowa wahad³a w otwartej pêtli');
ylabel('K¹t [rad]'), xlabel('Czas [s]');

%% Regulator PID dla phi - kata nachylenia wahadla
% http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlPID
Kp = 100;
Ki = 1;
Kd = 20;
C = pid(Kp,Ki,Kd);      % regulator PID
T = feedback(G_phi,C);  % transmitancja ukladu zamknietego z PID

%% Strojenie
t=0:0.01:10;
figure;
impulse(T,t);
title('OdpowiedŸ impulsowa k¹ta wahad³a pod kontrol¹ PID: Kp = 100, Ki = 1, Kd = 20');
ylabel('K¹t [rad]'), xlabel('Czas [s]');
axis([0 1 -0.06 0.1]);

%% Regulator PID dla x - pozycji kol
% potrzebny model w Simulinku nieliniowy - zachowanie linearyzacji x
% zupelnie inne
% 
% Kpx = 100;
% Kix = 10;
% Kdx = 20;
% Cx = pid(Kpx, Kix, Kdx);
% T3 = feedback(G_x, Cx);
% 
% figure;
% t=0:0.01:10;
% impulse(T3, t);

%% Pozycja kol robota
figure;
t=0:0.01:5;
T2 = feedback(1, G_phi*C)*G_x;
impulse(T2, t);
title('OdpowiedŸ impulsowa po³o¿enia kó³ robota pod kontrol¹ PID: Kp = 100, Ki = 1, Kd = 20');
ylabel('K¹t [rad]'), xlabel('Czas [s]');

%pzplot(sys_tf); title('Zera i bieguny uk³adu otwartego');
%pzplot(T); title('Zera i bieguny uk³adu zamknietego');