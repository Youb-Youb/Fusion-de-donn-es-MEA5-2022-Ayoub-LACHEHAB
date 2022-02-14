close all
clear all
clc

if ~isempty(instrfind)
   fclose(instrfind);
   delete(instrfind);
end
dimensions=[0.4;0.5;0.1];
TIME=[];
% draw data
figure,
grid();
view( -70, 40 );
axis vis3d equal;
stop=uicontrol('style','radiobutton','position',[100 50 100,20],...
    'string','STOP','value',0,'backgroundcolor','c','foregroundcolor','k');

xlabel('X')
ylabel('Y')
zlabel('Z')

% SERIAL PORT CONFIG
IMU = serial('com16');
set(IMU,'BaudRate',9600);
IMU.Terminator = ';';
fopen(IMU);
t=1;

while t<inf && get(stop,'value')==0
   TIME=[TIME t];
    % Data quaternions calibred with Madgwick method
    Data(t, 1) = str2double(fgetl(IMU));
    Data(t, 2) = str2double(fgetl(IMU));
    Data(t, 3) = str2double(fgetl(IMU));
    
    % Pur Data of Magnetometer
    
    %affect Data Magnetometer to our variables declaration
    magx(t)= Data(t, 1);
    magy(t)= Data(t, 2);
    magz(t) = Data(t, 3);
    
    if t>9
        [ center, radii, evecs, v, chi2 ] = ellipsoid_fit( [magx(:) magy(:) magz(:)], '');
        plot3( magx(:), magy(:), magz(:), 'ro' );
        hold on;
        %draw fit
        mind(1) = min( magx(:)');
        mind(2) = min( magy(:)');
        mind(3) = min( magz(:)');
        maxd(1) = max( magx(:)');
        maxd(2) = max( magy(:)');
        maxd(3) = max( magz(:)');
        
        offset(t-9,1)=maxd(1)-mind(1)/2;
        offset(t-9,2)=maxd(1)-mind(1)/2;
        offset(t-9,3)=maxd(1)-mind(1)/2;

        nsteps = 50;
        step = ( maxd - mind ) / nsteps;
        [ x, y, z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );

    Ellipsoid = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
              2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
              2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
    p = patch( isosurface( x, y, z, Ellipsoid, -v(10) ) );
        hold off;
        set( p, 'FaceColor', 'c', 'EdgeColor', 'non' );
        camlight;
        lighting phong;
    end
        t=t+1;
        drawnow;
end
fprintf( 'Ellipsoid center: %.5g %.5g %.5g\n', center );
fprintf( 'Ellipsoid radii: %.5g %.5g %.5g\n', radii );
fprintf( 'Ellipsoid evecs:\n' );
fprintf( '%.5g %.5g %.5g\n%.5g %.5g %.5g\n%.5g %.5g %.5g\n', ...
    evecs(1), evecs(2), evecs(3), evecs(4), evecs(5), evecs(6), evecs(7), evecs(8), evecs(9) );
fprintf( 'Algebraic form:\n' );
fprintf( '%.5g ', v );
fprintf( '\nAverage deviation of the fit: %.5f\n', sqrt( chi2 / size( magx', 1 ) ) );
fprintf( '\n' );