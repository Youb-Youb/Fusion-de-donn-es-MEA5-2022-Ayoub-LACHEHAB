close all
clear all
clc
clf
dimensions=[0.5;0.4;0.3];
Q0=[1 0 0 0];

%Initialization For variables
BiaisGx=0;BiaisAx=0;BiaisMx=0;
BiaisGy=0;BiaisAy=0;BiaisMy=0;
BiaisGz=0;BiaisAz=0;BiaisMz=0;
x=0;
TIME=[];
Gx=0;Ax=0;Mx=0;
Gy=0;Ay=0;My=0;
Gz=0;Az=0;Mz=0;
EcaData=[];
nbCalib=100;
dt=0.01;
figure(1);
grid();
L=dimensions(1);l=dimensions(2);h=dimensions(3);
axis([-L L -L L -L L]*1.2)
axis equal
view(24,31)

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

stop=uicontrol('style','radiobutton','position',[100 50 100,20],...
    'string','Stop simulation','value',0,'backgroundcolor','c','foregroundcolor','k');

xlabel('X')
ylabel('Y')
zlabel('Z')

line([0 1],[0 0],[0 0],'color','r','linewidth',1);
line([0 0],[0 1],[0 0],'color','b','linewidth',1)
line([0 0],[0 0],[0 1],'color','g','linewidth',1)

ptrG=line([0 0],[0 0],[0 0],'color','k','linewidth',1);

% SERIAL PORT CONFIG
IMU = serial('com16');
set(IMU,'BaudRate',9600);
IMU.Terminator = ';';
fopen(IMU);
t=1;
%161;
%x_xl[bits];y_xl[bits];z_xl[bits];
%x_gy[bits];y_gy[bits];z_gy[bits];
%x_mag[bits];y_mag[bits];z_mag[bits];
%temp[bits];[opt::temp_ext[bits]]\
[ptrch,A0_S,B1_S,B2_S,B3_S]=Init_Dessin_Robot(Q0,dimensions);

for i = 0: nbCalib    
    if(str2double(fgetl(IMU))== 161)         %First indicated of Pur frame data output
        EcaData(i, 1) = str2double(fgetl(IMU));
        EcaData(i, 2) = str2double(fgetl(IMU));
        EcaData(i, 3) = str2double(fgetl(IMU));
        EcaData(i, 4) = str2double(fgetl(IMU));
        EcaData(i, 5) = str2double(fgetl(IMU));
        EcaData(i, 6) = str2double(fgetl(IMU));
        EcaData(i, 7) = str2double(fgetl(IMU));
        EcaData(i, 8) = str2double(fgetl(IMU));
        EcaData(i, 9) = str2double(fgetl(IMU));
        EcaData(i, 10) = str2double(fgetl(IMU));
        
        BiaisAx=BiaisAx+EcaData(i,1);
        BiaisAy=BiaisAy+EcaData(i,2);
        BiaisAz=BiaisAz+EcaData(i,3);
        
        BiaisGx=BiaisGx+EcaData(i,4);
        BiaisGy=BiaisGy+EcaData(i,5);
        BiaisGz=BiaisGz+EcaData(i,6);
        
        
        BiaisMx=BiaisMx+EcaData(i,7);
        BiaisMy=BiaisMy+EcaData(i,8);
        BiaisMz=BiaisMz+EcaData(i,9);
    end 
end

while t<inf && get(stop,'value')==0
    TIME=[TIME t];
    if(str2double(fgetl(IMU))== 161)         %First indicated of Pur frame data output         
        EcaData(t,1) = str2double(fgetl(IMU));
        EcaData(t,2) = str2double(fgetl(IMU));
        EcaData(t,3) = str2double(fgetl(IMU));
        EcaData(t,4) = str2double(fgetl(IMU));
        EcaData(t,5) = str2double(fgetl(IMU));
        EcaData(t,6) = str2double(fgetl(IMU));
        EcaData(t,7) = str2double(fgetl(IMU));
        EcaData(t,8) = str2double(fgetl(IMU));
        EcaData(i, 9) = str2double(fgetl(IMU));
        EcaData(i, 10) = str2double(fgetl(IMU));
        
        Ax=EcaData(t,1);
        Ay=EcaData(t,2);
        Az=EcaData(t,3);
        
        Gx=EcaData(t,4);
        Gy=EcaData(t,5);
        Gz=EcaData(t,6);
        
        Mx=EcaData(t,7);
        My=EcaData(t,8);
        Mz=EcaData(t,9);


    end
    
    TabGyro(t,1)=Gx;TabGyro(t,2)=Gy;TabGyro(t,3)=Gz;
    Gyroscope(t,1) = Gx-(BiaisGx /nbCalib);Gyroscope(t,2) = Gy-(BiaisGy /nbCalib);Gyroscope(t,3) = Gz-(BiaisGz /nbCalib); 
    Magnetometer(t,1)=Mx;Magnetometer(t,2)=My;Magnetometer(t,3)=Mz;
    Accelerometer(t,1)=Ax;Accelerometer(t,2)=Ay;Accelerometer(t,3)=Az;

    % Process sensor data through algorithm
    AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
    % AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
    
    [ptrch]=Dessin_Robot(quaternion(t, :),ptrch,dimensions);
    
    drawnow
    t=t+1;
end   
fclose(IMU);

euler = quaternConj(quaternion) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(TIME, euler(:,1), 'r');
plot(TIME, euler(:,2), 'g');
plot(TIME, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;