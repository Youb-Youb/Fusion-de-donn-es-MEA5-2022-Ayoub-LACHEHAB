close all
clear all
clc
clf

%Initialization For variables
BiaisGx=0;
BiaisGy=0;
BiaisGz=0;

x=0;
TIME=[];
Gx=0;Ax=0;Mx=0;
Gy=0;Ay=0;My=0;
Gz=0;Az=0;Mz=0;
EcaData=[];
nbCalib=100;
dt=0.01;
figure(1);

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

stop=uicontrol('style','radiobutton','position',[100 50 100,20],...
    'string','Stop simulation','value',0,'backgroundcolor','c','foregroundcolor','k');

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
%temp[bits];[opt::m[bits]]\

%for i = 1: nbCalib    
        %EcaData(i, 1) = str2double(fgetl(IMU));
        %EcaData(i, 2) = str2double(fgetl(IMU));
        %EcaData(i, 3) = str2double(fgetl(IMU));
        %EcaData(i, 4) = str2double(fgetl(IMU));
        %EcaData(i, 5) = str2double(fgetl(IMU));
        %EcaData(i, 6) = str2double(fgetl(IMU));
        %EcaData(i, 7) = str2double(fgetl(IMU));
        %EcaData(i, 8) = str2double(fgetl(IMU));
        %EcaData(i, 9) = str2double(fgetl(IMU));
        
        %BiaisGx=BiaisGx+EcaData(i,1);
        %BiaisGy=BiaisGy+EcaData(i,2);
        %BiaisGz=BiaisGz+EcaData(i,3);
        %end

while t<inf && get(stop,'value')==0
        TIME=[TIME t];       
        EcaData(t,1) = str2double(fgetl(IMU));
        EcaData(t,2) = str2double(fgetl(IMU));
        EcaData(t,3) = str2double(fgetl(IMU));
        EcaData(t,4) = str2double(fgetl(IMU));
        EcaData(t,5) = str2double(fgetl(IMU));
        EcaData(t,6) = str2double(fgetl(IMU));
        EcaData(t,7) = str2double(fgetl(IMU));
        EcaData(t,8) = str2double(fgetl(IMU));
        EcaData(t, 9)= str2double(fgetl(IMU));
 
        Ax=EcaData(t,4);
        Ay=EcaData(t,5);
        Az=EcaData(t,6);
        
        Gx=EcaData(t,1);
        Gy=EcaData(t,2);
        Gz=EcaData(t,3);
        
        Mx=EcaData(t,7);
        My=EcaData(t,8);
        Mz=EcaData(t,9);
    
        TabGyro(t,1)=Gx;TabGyro(t,2)=Gy;TabGyro(t,3)=Gz;
        TabMagn(t,1)=Mx;TabMagn(t,2)=My;TabMagn(t,3)=Mz;
        TabAlero(t,1)=Ax;TabAlero(t,2)=Ay;TabAlero(t,3)=Az;

        %TabBiais(t,1) = Gx-(BiaisGx /nbCalib);TabBiais(t,2) = Gy-(BiaisGy /nbCalib);TabBiais(t,3) = Gz-(BiaisGz /nbCalib); 
        TabBiais(t,1) = Gx;TabBiais(t,2) = Gy;TabBiais(t,3) = Gz; 

        Roll(1)=0;Pitch(1)=0;Yaw(1)=0; 
        Vx(1)=0;Vy(1)=0;Vz(1)=0;
        Px(1)=0;Py(1)=0;Pz(1)=0;
    
    h=1/100;
    if t>1
        Roll(t) =Roll(t-1)+ h/2*(TabBiais(t-1,1)+TabBiais(t,1));
        Pitch(t) =Pitch(t-1)+ h/2*(TabBiais(t-1,2)+TabBiais(t,2));
        Yaw(t) =Yaw(t-1)+ h/2*(TabBiais(t-1,3)+TabBiais(t,3));
    end    

    subplot(3,1,1);
    title('Gyroscope x without offset');
    % plot(TabBiais(:,1),'r');
    hold on
    plot(TIME,TabGyro(:,1),'b');
    subplot(3,1,2);
    title('Gyroscope y without offset')
    %plot(TIME,TabBiais(:,2),'r');
    hold on
    plot(TIME,TabGyro(:,2),'b');
    subplot(3,1,3);
    %plot(TIME,TabBiais(:,3),'r');
    hold on
    plot(TIME,TabGyro(:,3),'b');  
    title('Gyroscope z without offset')
    hold off;
    t=t+1;
    drawnow
    clc
end
figure(2);
subplot(3,1,1);
title('Roll');
hold on
plot(TIME,Roll,'r');
subplot(3,1,2);
title('Pitch')
hold on
plot(TIME,Pitch,'r');
subplot(3,1,3);
title('Yaw')
hold on;
plot(TIME,Yaw,'r');
figure(4)
subplot(3,1,1);
title('Ax');
hold on
plot(TIME,TabAlero(:,1),'b');
subplot(3,1,2);
title('Ay')
hold on
plot(TIME,TabAlero(:,2),'b');
subplot(3,1,3);
plot(TIME,TabAlero(:,3),'b');  
title('Az')
hold off;
% figure(3);
% subplot(3,1,1);
% title('Velocity x');
% hold on
% plot(TIME,Vx,'r');
% subplot(3,1,2);
% title('Velocity y')
% hold on
% plot(TIME,Vy,'r');
% subplot(3,1,3);
% title('Velocity z')
% hold on;
% plot(TIME,Vz,'r');
% figure(4);
% subplot(3,1,1);
% title('Position x');
% hold on
% plot(x,Px,'r');
% subplot(3,1,2);
% title('Position y')
% hold on
% plot(x,Py,'r');
% subplot(3,1,3);
% title('Position z')
% hold on;
% plot(x,Pz,'r');