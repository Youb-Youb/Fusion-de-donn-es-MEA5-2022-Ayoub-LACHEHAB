close all
clear all
clc
%
RAD_TO_DEG = 57.2957795;
%intialization des varibles declarations 
dt = 0.01;
a = [0.8,0,0];
b = [0,0.8,0];
c= [0,0,0.8];
Q0=[1 0 0 0];
Gyrox=zeros(1);
Gyroy=zeros(1);
Gyroz=zeros(1);
TIME=[];
%close port communication 
if ~isempty(instrfind)
   fclose(instrfind);
end
%Print the text to request the type of Data simulation
text='Request : Do you want to plot the quaternion simulator ? :';
QuaternionRequest = input(text,'s'); 

% La configuration de terminal 
IMU = serial('com4');
set(IMU,'BaudRate',9600);
IMU.Terminator = ';';
fopen(IMU);

if QuaternionRequest=="yes"
    figure('Name','the quaternion simulator');
    clf;
    hold on
    grid();
    %Délclaration des 3 axes
    pt1=line([0 0],[0 0],[0 0],'color','r','linewidth',1);
    pt2=line([0 0],[0 0],[0 0],'color','b','linewidth',1);
    pt3=line([0 0],[0 0],[0 0],'color','green','linewidth',1);
    
    stop=uicontrol('style','radiobutton','position',[100 50 100,20],...
        'string','Plot quaternions','value',0,'backgroundcolor','c','foregroundcolor','k');
    
    dimensions=[0.4;0.5;0.1];
    L=dimensions(1);l=dimensions(2);h=dimensions(3);
    axis([-L L -L L -L L]*2)
    axis equal
    view(24,31)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    line([0 0.8],[0 0],[0 0],'color','r','linewidth',1);
    line([0 0],[0 0.8],[0 0],'color','b','linewidth',1)
    line([0 0],[0 0],[0 0.8],'color','green','linewidth',1)
    ptrG=line([0 0],[0 0],[0 0],'color','k','linewidth',1);
    %Initialisation du Dessin parallélépipède
    [ptrch,A0_S,B1_S,B2_S,B3_S]=Init_Dessin_Robot(Q0,dimensions);
    pt=title('');
elseif QuaternionRequest=="no"
    text1='Request : Do you want to plot the euler angles ? :';
    EulerRequest = input(text1,'s');
    text2='Request : Euler angles coming From STM32  or Matlab ? :';
    TypePlotRequest = input(text2,'s');
    figure('Name', 'Euler Angles');
    hold on;
    stop=uicontrol('style','radiobutton','position',[100 50 100,20],...
        'string','plot quaternions','value',0,'backgroundcolor','c','foregroundcolor','k');
end
%Start time
t=1;
while t<inf && get(stop,'value')==0
   TIME=[TIME t];
   Quat=zeros(t,4);
   EulerAngle=zeros(t,3);
   
   % Affecter les valeurs aux variables recues
    while (str2double(fgetl(IMU))== 99) %% le début de trame 
        % Pur Data of quaternions
        Quat(t, 1) = str2double(fgetl(IMU));
        Quat(t, 2) = str2double(fgetl(IMU));
        Quat(t, 3) = str2double(fgetl(IMU));
        Quat(t, 4) = str2double(fgetl(IMU));
        EulerAngle(t,1)=str2double(fgetl(IMU));
        EulerAngle(t,2)=str2double(fgetl(IMU));
        EulerAngle(t,3)=str2double(fgetl(IMU));
        Gyrox=str2double(fgetl(IMU));
        Gyroy=str2double(fgetl(IMU));
        Gyroz=str2double(fgetl(IMU));
        break;
    end
    GyroXt(t)=Gyrox;
    GyroYt(t)=Gyroy;
    GyroZt(t)=Gyroz;
    Q(t, 1) = Quat(t, 1);
    Q(t, 2) = Quat(t, 2);
    Q(t, 3) = Quat(t, 3);
    Q(t, 4)=  Quat(t, 4);
    Rollt(t)=EulerAngle(t,1);
    Pitcht(t)=EulerAngle(t,2);
    Yawt(t)=EulerAngle(t,3);
    %Convertir les quaternions aux angles d'Euler
    [Roll(t) Pitch(t) Yaw(t)] = Convert_QuaternionToEuler(Q(t, :)); 
    %Appel du filtre de kalman
    YawFK(t) = kalmanFilter(dt, Yaw(t), GyroZt(t));

    %les vecteurs quarternions 
    q = quaternion(Q(t, :));
    %Calculer les nouveaux points en fonction du vecteur quarternions
    rP = rotatepoint(q,[a;b;c]);
    if QuaternionRequest=="yes"
        %Mise à jour du dessin 
        [ptrch]=Dessin_Robot(Q(t, :),ptrch,dimensions);
        set(pt1,'xdata',[0 rP(1,1)],'ydata',[0 rP(1,2)],'zdata',[0 rP(1,3)],'color','r','linewidth',1);
        set(pt2,'xdata',[0 rP(2,1)],'ydata',[0 rP(2,2)],'zdata',[0 rP(2,3)],'color','b','linewidth',1);
        set(pt3,'xdata',[0 rP(3,1)],'ydata',[0 rP(3,2)],'zdata',[0 rP(3,3)],'color','green','linewidth',1);
        
        %Calculer l'angle crée entre l'axe fixe et mobile '2ème facon'
        angle(t,1) = atan2(norm(cross([0.8 0 0],rP(1,1:3))), dot([0.8 0 0],rP(1,1:3)))*RAD_TO_DEG;
        angle(t,2) = atan2(norm(cross([0 0.8 0],rP(2,1:3))), dot([0 0.8 0],rP(2,1:3)))*RAD_TO_DEG;
        angle(t,3) = atan2(norm(cross([0 0 0.8],rP(3,1:3))), dot([0 0 0.8],rP(3,1:3)))*RAD_TO_DEG;
    elseif QuaternionRequest=="no"
        if EulerRequest=="yes"
            if TypePlotRequest=="STM32" %Lire les angles d'euler caclulés dans le logiciel
                plot(Pitcht(:), 'r');
                hold on;
                plot(Rollt(:), 'g');
                plot(Yawt(:), 'b');
                title('Euler angles');
                xlabel('Time (s)');
                ylabel('Angle (deg)');
                legend('\phi', '\theta', '\psi');
                %hold off;   
            elseif TypePlotRequest=="Matlab"      %Lire les angles d'euler calculés par Matlab
                plot(Roll(:), 'r');
                hold on;
                plot(Pitch(:), 'g');
                plot(Yaw(:), 'b');
                title('Euler angles');
                xlabel('Time (s)');
                ylabel('Angle (deg)');
                legend('\phi', '\theta', '\psi');
                %hold off;
            end
        elseif EulerRequest=="no"
        end
    end
    drawnow
    maxtime=t;
    t=t+1;
    index=size(Q);
end
fclose(IMU);
sizeG=size(angle(:));
x = 1:sizeG;

% Plot les angles d'euler et le vecteur quaternion
hold on;
subplot(2,2,1)
plot(Q(:,1),'b')
maxy= max(Q(:,1));
miny=min(Q(:,1));
line([maxtime maxtime],[miny maxy],'Color','green','LineStyle','-')
line([0 maxtime],[maxy maxy],'Color','red','LineStyle','--')
line([0 maxtime],[miny miny],'Color','red','LineStyle','--')
legend(sprintf('Q0 = %0.3f',Q(index(1),1)),sprintf('Ecart = %0.6f',maxy-miny))
title('Q0')
subplot(2,2,2)
hold on
plot(Q(:,2),'b')
maxy= max(Q(:,2));
miny=min(Q(:,2));
line([maxtime maxtime],[miny maxy],'Color','green','LineStyle','-')
line([0 maxtime],[maxy maxy],'Color','red','LineStyle','--')
line([0 maxtime],[miny miny],'Color','red','LineStyle','--')
legend(sprintf('Q1 = %0.3f',Q(index(1),2)),sprintf('Ecart = %0.6f',maxy-miny));
title('Q1')
subplot(2,2,3)
hold on
plot(Q(:,3),'b')
maxy= max(Q(:,3));
miny=min(Q(:,3));
line([maxtime maxtime],[miny maxy],'Color','green','LineStyle','-')
line([0 maxtime],[maxy maxy],'Color','red','LineStyle','--')
line([0 maxtime],[miny miny],'Color','red','LineStyle','--')
legend(sprintf('Q2 = %0.3f',Q(index(1),3)),sprintf('Ecart = %0.6f',maxy-miny))
title('Q2')
subplot(2,2,4)
hold on
plot(Q(:,4),'b')
maxy= max(Q(:,4));
miny=min(Q(:,4));
line([maxtime maxtime],[miny maxy],'Color','green','LineStyle','-');
line([0 maxtime],[maxy maxy],'Color','red','LineStyle','--');
line([0 maxtime],[miny miny],'Color','red','LineStyle','--');
legend(sprintf('Q3 = %0.3f',Q(index(1),4)),sprintf('Ecart = %0.6f',maxy-miny));
title('Q3')



figure('Name', 'STM32 Euler angle');
plot(TIME,Pitcht(:), 'r');
hold on;
plot(TIME,Rollt(:), 'g');
plot(TIME,Yawt(:), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off; 

figure('Name', 'Matlab Euler angle');
plot(TIME,Roll(:), 'r');
hold on;
plot(TIME,Pitch(:), 'g');
plot(TIME,Yaw(:), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

figure('Name', 'angle');
plot(TIME, angle(:,1), 'r');
hold on;
plot(TIME, angle(:,2), 'b');
plot(TIME, angle(:,3), 'g');
legend('\phi', '\theta', '\psi');
hold off;


figure('Name', 'Filtre de Kalman');
%plot(TIME,RollFK(:), 'r');
%hold on;
%plot(TIME,PitchFK(:), 'g');
plot(TIME,YawFK(:), 'b');
title('Yaw');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\psi');
hold off;

% figure(4);
% subplot(2,1,1);
% [localmax] = islocalmax(angle(:));
% stem(x(localmax),angle(localmax));
% title('max');
% subplot(2,1,2);
% [localmin]= islocalmin(angle(:));
% stem(x(localmin),angle(localmin));
% title('min');