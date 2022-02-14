function rollKF = kalmanFilter(dt, newAngle, newRate)

%initialization of variables
persistent angle0;
persistent P0;
R= 3.0462e-06; %For MPU9250 error variance
Q_angle = 0.0001;
Q_bias = 0.003;

%Process Noise Convariance Matrix
Q = [Q_angle 0; 0 Q_bias] * dt;
A = [1  -dt; 0 1];
Ht = [1 0];
bias0 = 0.0;

%Kalman algo
rate0 = newRate - bias0;
if isempty(angle0); angle0 = 0;  
else angle0 = angle0 + (dt * rate0);
end 
rollKF = angle0;
% --------------------------the algorithm of predition --------------------------------% 
if isempty(P0); P0 = [0, 0; 0, 0];
else P0 = A*P0*A' + Q;
end 
P0_temp = P0;
% ---------------------------------Update algorithm -----------------------------------% 

%kalman gain calculation 
S0 = Ht*P0*Ht' + R; %Estimation of the covariance of the error
Kt = (P0_temp * Ht')/S0;
y0 = newAngle - rollKF; 

rollKF = rollKF + Kt(1) * y0;
bias0 = rollKF + Kt(2) * y0;
angle0 = rollKF;

%correction&innovation
I = eye(2);
P0_temp = (I - Kt*Ht)*P0_temp;
P0 = P0_temp;
