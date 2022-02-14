
%function [Et,Z] = ImuCalibrationKalman(T,gx,gy,gz)
N=1000;%sampling time
T = 0.00675;
dt=0.001;
%t=2% %bande test
  gx=1;
  gy=1;
  gz=1;
%State Vector 
E=[gx gy gz 0 0 0]';
%transition matrice 

A =[[1,0,0,dt,0,0];
    [0,1,0,0,dt,0];
    [0,0,1,0,0,dt];
    [0,0,0,1,0,0];
    [0,0,0,0,1,0];
    [0,0,0,0,0,1]];

sizeA = size(A);

% for k=2:N
%      E(:,k)=A*E(:,k-1); 
% end
%Oberservation matrice of X,Y and Z "to define whitch measure be taken"
H=[[1,0,0,0,0,0];
    [0,0,0,0,0,0];
    [0,0,0,0,0,0]];

%Measurement noise model "related to the evolution of my system"
Q=[[1,0,0,0,0,0];
    [0,1,0,0,0,0];
    [0,0,1,0,0,0];
    [0,0,0,1,0,0];
    [0,0,0,0,1,0];
    [0,0,0,0,0,1]];

R=eye(3); % Measurement matrix with noise " "find it in the datasheet of imu"
%R=[3.0462e-06 3.0462e-06 3.0462e-06]' ; %For MPU9250

Z=H*E+R;
P=ones(sizeA(1));


% --------------------------the algorithm of predition --------------------------------% 

E=A*E;  %update of state matrix
P=A*P*A'+Q; %Estimation of the covariance of the error

% ---------------------------------Update algorithm -----------------------------------% 
Z=H*E+R;
%kalman gain calculation 
K=P*H'/(H*P*H'+R);
%correction&innovation
Id=eye(sizeA(1));
P=(Id-K*H)*P;
Et=E+K*(Z-H*E);

%end
 