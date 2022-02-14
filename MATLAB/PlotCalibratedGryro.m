clear all
clc
close all

%Initialization of variables
TIME = [];
Gx=0;Gy=0;Gz=0;
%Check if the port com is closed
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% SERIAL PORT CONFIG
IMU = serial('com4');
set(IMU,'BaudRate',9600);
IMU.Terminator = ';';
fopen(IMU);
t=1;

while t<300 
    TIME = [TIME t];
    while (str2double(fgetl(IMU))== 99)
        Data(t, 1) = str2double(fgetl(IMU));
        Data(t, 2) = str2double(fgetl(IMU));
        Data(t, 3) = str2double(fgetl(IMU));
        Data(t, 4) = str2double(fgetl(IMU));
        Data(t, 5) = str2double(fgetl(IMU));
        Data(t, 6) = str2double(fgetl(IMU));
        Gx=Data(t, 1);Gy=Data(t, 2);Gz=Data(t,3);
    disp(Gx);
        break;
    end
    TabGyro(t,1)=Gx;TabGyro(t,2)=Gy;TabGyro(t,3)=Gz;
    Roll(1)=0;Pitch(1)=0;Yaw(1)=0; 

    h=1/100;
    if t>1
        Roll(t) =Roll(t-1)+ h/2*(TabGyro(t-1,1)+TabGyro(t,1));
        Pitch(t) =Pitch(t-1)+ h/2*(TabGyro(t-1,2)+TabGyro(t,2));
        Yaw(t) =Yaw(t-1)+ h/2*(TabGyro(t-1,3)+TabGyro(t,3));
    end  
    t=t+1;
    drawnow
    clc
end
figure(1);
subplot(3,1,1);
title('Acceleration x');
hold on
plot(TIME,Data(:,1),'b');
subplot(3,1,2);
title('Acceleration y')
hold on
plot(TIME,Data(:,2),'b');
subplot(3,1,3);
hold on
plot(TIME,Data(:,3),'b');  
title('Acceleration z')
hold off;

figure(2);
subplot(3,1,1);
title('Gyroscope x');
hold on
plot(TIME,Data(:,4),'b');
subplot(3,1,2);
title('Gyroscope y')
hold on
plot(TIME,Data(:,5),'b');
subplot(3,1,3);
hold on
plot(TIME,Data(:,6),'b');  
title('Gyroscope z')
hold off;

figure(3);
subplot(3,1,1);
title('Gyroscope x');
hold on
plot(TIME,Roll,'r');
subplot(3,1,2);
title('Gyroscope y')
hold on
plot(TIME,Pitch,'r');
subplot(3,1,3);
title('Gyroscope z')
hold on;
plot(TIME,Yaw,'r');