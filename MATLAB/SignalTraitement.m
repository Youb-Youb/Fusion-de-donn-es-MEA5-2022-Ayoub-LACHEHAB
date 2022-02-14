close all
clear all
clc
clf
dt=0.01;
figure(1);
grid();

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

stop=uicontrol('style','radiobutton','position',[100 50 100,20],...
    'string','Stop simulation','value',0,'backgroundcolor','c','foregroundcolor','k');
% SERIAL PORT CONFIG
IMU = serial('com16');
set(IMU,'BaudRate',115200);
IMU.Terminator = '|';
out2 = instrfind({'Port','BaudRate'},{'COM16',115200});
fopen(IMU);
t=1;
TIME = [];
FxE = [1; 0];
Fx = [1; 0];
index=3;
Pt = plot(FxE,'-r');
N = 5;
while t<inf && get(stop,'value')==0 
    TIME = [TIME t];
    
    Data(t, 1) = str2double(fgetl(IMU));
    Data(t, 2) = str2double(fgetl(IMU));
    Data(t, 3) = str2double(fgetl(IMU));
        
    G(t, 1) = Data(t, 1);
    G(t, 2) = Data(t, 2);
    G(t, 3) = Data(t, 3);
    hold on;
    plot(G(:, index),'b');
    if(mod(t,N) == 0)
        Fx = Algo_Echantionnage(t,N,G(:, 3), Fx, TIME);
        FxE = interp1(Fx(1, :), Fx(2, :), TIME);
        delete(Pt);
        Pt = plot(FxE,'-r');
    end
    drawnow
    t=t+1;
end
sizeG=size(G(:, index));
x = 1:sizeG;
figure(2)
subplot(3,1,1);
localmax = islocalmax(G(:, index));
plot(x,G(:, index),x(localmax),G(localmax, index),'*','MarkerSize',8);
title('signal with max');
subplot(3,1,2);
localmin = islocalmin(G(:, index));
plot(x,G(:, index),x(localmin),G(localmin, index),'*','MarkerSize',8);
title('signal with min');

Ak=abs(fft(G(:, index)))/length(G(:, index)); % calcul du spectre d’amplitude
fs=8000;
k=0:1:length(G(:, index))-1; % Génération de l’indice des fréqu.
f=k*fs/length(G(:, index)); % conversion en Hz
subplot(3,1,3); plot(f,Ak);
title('Spectre d''amplitude');  % trace du spectre d’amplitude 

figure(3);
%[Maxima,MaxIdx] = findpeaks(G(:, index));
subplot(2,1,1);
[localmax] = islocalmax(G(:, index));
stem(x(localmax),G(localmax, index));
title('signal with max');
subplot(2,1,2);
[localmin]= islocalmin(G(:, index));
stem(x(localmin),G(localmin, index));
title('signal with min');

function [Nu,F] = TFourier(f,dt)
N=max(size(f)); 
Nu=(0:N-1)/(N*dt);
F=abs(fft(f));
end