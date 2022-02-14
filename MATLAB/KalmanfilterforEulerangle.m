clear all;
close all;
clc
%Initialization Variables 
freq=20.0;		%sample frequency in Hz
T=1/freq;
t=1;
if ~isempty(instrfind)
   fclose(instrfind);
   delete(instrfind);
end
% SERIAL PORT CONFIG
IMU = serial('com4');
set(IMU,'BaudRate',115200);
IMU.Terminator = ';';
fopen(IMU);
figure(1);

while t<inf
    disp(t)
    [datax(t), datay(t), dataz(t)] = RecupDataEcarobotibs(IMU,'Gyro');
    G=[datax(t) datay(t) dataz(t)];
    VarG=[var(datax(:)) var(datay(:)) var(dataz(:))]; %Varience of Gyro data
    n=size(datax(:),1);
    X_p=zeros(3,n);
    X_e=zeros(3,n); %state vector
    F=eye(3);  %Control matrix
    B=T*eye(3); %Input vector
    %Initial vetor of state vector 
    [Pitch(t),Yaw(t),Roll(t)] = RecupDataEcarobotibs(IMU,'Eulerangle');
%     Pitch0=Pitch(1);
%     Yaw0=Yaw(t);
%     Roll0=Roll(t);
%     X_e(:,1) = [Pitch0 Yaw0 Roll0]';
%     %Initial value of state model covariance
%     P_e=[100 0 0; 0 100 0;0 0 225];
%     [Ax(t),Ay(t),Az(t)] = RecupDataEcarobotibs(IMU,'acelero');
%     z(:,t)=[Ax(t),Ay(t),Az(t)];
%     % Observation functions
%     h = [-sind(Pitch(t)) cosd(Pitch(t)).*sind(Roll(t)) cosd(Pitch(t)).*cosd(Roll(t))]';
%     % Jacobian matrix of the observation functions
%     sizeh = max(size(h));
%     H = zeros(3,3,sizeh); %the gradiant 
%     for i =1:t
%         H(:,:,i) = [0 -cosd(Pitch(i)) 0
%         0 -sind(Pitch(i)).*sind(Roll(i)) cosd(Pitch(i)).*cosd(Roll(i))
%         0 -sind(Pitch(i)).*cosd(Roll(i)) -cosd(Pitch(i)).*sind(Roll(i))];
%     end
%     % ACC covariance measurement noise matrix
%     R = eye(3).*10^(-5);
%     % EKF loop
%     for j = 1:n
%         X_p(:,j) = F*X_e(:,j) + B*G'; % Predicted state vector
%         P_p = F*P_e*F' + VarG; % Predicted covariance matrix
%         e_t = z(:,j) - h;%H*x_p(:,t); % z(:,t) measured data from accelerometer
%         
%         S = H(:,:,j)*P_p*H(:,:,j)' + R;
%         K = P_p*H(:,:,j)'*(S)^-1; % Kalman gain
%         X_e(:,j+1) = X_p(:,j) + K*e_t; % Updated state vector
%         P_e = P_p - K*H(:,:,j)*P_p; % Updated covariance matrix
%     end
%     angles = X_e(:,1:end-1)';
%     t=t+1;
    plot(Roll(:));
    drawnow;
end