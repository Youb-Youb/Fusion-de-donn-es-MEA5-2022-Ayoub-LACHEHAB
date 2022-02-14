
clear t % Clears old time steps and
clear y % y values from previous runs
figure();
a=0; % Initial time
b=1; % Final time
N=10; % Number of time steps
y0=0; % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
y(1)=y0;
for n=1:N % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    y(n+1)=y(n)+h*f(t(n),y(n)); % Calls the function f(t,y)=dy/dt
end
plot(t,y)

% Include your own name
function y=f(t,y)
    y=3*t; % Defines the function f
end