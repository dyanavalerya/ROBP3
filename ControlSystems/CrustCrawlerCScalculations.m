
%%===============================
% System Requirements
% Here we set the desired requirements, how much overshoor is desired and
% the settling time of the system
Os= 5;
Ts = 2;

%%==================================
% Controller gain calculations
%First we calculate the dampening ratio
zeta = (-log(Os/100)*(sqrt(1/(pi^2+log(Os/100)^2))))

%Here kp and kv is calcualted based on the requirements for the system
kp= ((4/zeta)*(1/Ts))^2
kv = 2*zeta*sqrt(kp)

%%==================================
% Bodeplot and gain/phase margin
figure(1)

%Firstly we are writing up the Open loop tranfer function for the system
%We write the numerator and the denominator seperatly
num = [kp];
den = [1 kv 0];

%Here the numerator and denominator is put together as a transfer function
OpenLoopTf = tf(num,den) 

%Here we plot the bode plot for the open loop transfer function and find
%the phase and gain margin of the system
margin(OpenLoopTf)
grid on

%%===================================
% Step Response
% Here we create a figure of a step response on our closedloop transfer
% function.

figure(2)
%This step applies the golden rule to our openloop transfer function giving
%us the closed loop transfer function

sysCL = feedback(OpenLoopTf, 1)
%Here the closed loop transfer function is put through a step reponse and
%graphed.

step(sysCL)
%After this the infomation about it like settling time rise time etc. is
%being printed to the command window
stepinfo(sysCL)