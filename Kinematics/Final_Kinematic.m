close all
clear all
clc
syms theta1 theta2 theta3 L1 L2 L3
%% Forward Kinematics
% Here we do the Forward kinematics so we can compare my results from the
% inverse with the forward. If ther are the same then the equations and
% methods are correct. We use the x,y,z coordinates from forward and use
% them in the inverse calculations, compare the calculated thetas with the
% thetas i inserted into the forward equation. 

%############## Link Lengths ##############%
% You can outcomment the anything after theta1 and work symbolicly to get
% the general form of the equation.

L1=0.07;
L2=0.23;
L3=0.27;
%User degree input for forward kinematics
degreeInput1=110;
degreeInput2=180;
degreeInput3=23;

%############## Converters from degree to radians ##############%
% Converts user input into radians 
% You can outcomment the anything after theta1 and work symbolicly to get
% the general form of the equation.
theta1=degreeInput1*pi/180;
theta2=degreeInput2*pi/180;
theta3=degreeInput3*pi/180;


%############## Forward Kinematics transformation matrices ##############%
% 
  T01 = [cos(theta1),  sin(theta1), 0, 0
        -sin(theta1),  cos(theta1), 0, 0
                   0, 0, 1, L1
      0, 0,  0, 1];
  
  
 T12 =[ cos(theta2), sin(theta2), 0, 0
        0, 0,-1,  0
   -sin(theta2), cos(theta2), 0,  0
   0, 0, 0,  1];
  
  
 T23 =[ cos(theta3), sin(theta3), 0,L2 
         -sin(theta3), cos(theta2), 0,  0
       0, 0, 1,  0
       0, 0, 0,  1];
 
 TCP =[ 1, 0, 0, L3
        0, 1, 0,  0
        0, 0, 1, 0
        0, 0, 0,  1]; 


 % The final transformation matrix for the whole robot. From base to 
T0TCP=((T01*T12*T23*TCP))
TF=T0TCP ;
T2P = (T01)^-1*TF %we remove the base from the transformation 


 %% Inverse Kinematics 


%############## Inverse Kinematics computations ##############%
% Pulling x,y,z coordintas from the trasnformation matrix 
x = TF(1, 4);
y = TF(2, 4);
z = TF(3, 4);



theta1i = -atan2(TF(2,4),TF(1,4)) ;

% Intermediat calculation for theta computations. It gives the
% magnitude(length)  

Lx = sqrt(T2P(3,4)^2+T2P(1,4);

% Intermediat calculations for theta2i
beta= acos((L2^2+Lx^2-L3^2)/(2*L2*Lx));
alpha= atan2(T2P(3,4),T2P(1,4));
beta*180/pi;
alpha*180/pi;

theta2i = -( beta + alpha);
theta3i=acos(-(L3^2+L2^2-Lx^2)/(2*L3*L2)); %Elbow / theta3

% Convert radians to degrees for the final display.
theta1Degree = theta1i*180/pi
theta2Degree = theta2i*180/pi
theta3Degree = theta3i*180/pi

%% For testing 
% The test runs the output degrees throught the forward kinematics.
% The resulting matriecs can be compared.

theta1Degree = theta1i; %*180/pi;
theta2Degree = theta2i; %*180/pi;
theta3Degree = theta3i; %*180/pi

TB1T=(TDH(0, 0, L1, theta1i));
T12T=(TDH(pi/2, 0, 0, theta2i));
T23T=(TDH(0, L2, 0, theta3i));
TCPT=(TDH(0, L3, 0, 0));


 % The final transformation matrix for the whole robot. From base to 
T0TCPT=(TB1T*T12T*T23T*TCPT)
TFTT=T0TCPT ;
T2PT = (TB1T)^-1*TFTT %we remove the base from the transformation 






%% Visual Model 

L(1)=Link([0          0     0           pi/2  ]);
L(2)=Link([0           0    L2          0      ]);
L(3)=Link([0                0    L3          0 ]);       
 Rob=SerialLink(L);
 Rob.name = "crustCrawler";
 Rob.teach
 