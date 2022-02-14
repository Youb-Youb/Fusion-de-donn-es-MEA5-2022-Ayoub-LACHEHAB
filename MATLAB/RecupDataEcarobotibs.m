%x_xl[bits];y_xl[bits];z_xl[bits];
%x_gy[bits];y_gy[bits];z_gy[bits];
%x_mag[bits];y_mag[bits];z_mag[bits];
%temp[bits];[opt::temp_ext[bits]]\

function [datax, datay, dataz] = RecupDataEcarobotibs(IMU,typeData)

    if(str2double(fgetl(IMU))== 99)         %First indicated of Pur frame data output         
        EcaData(1) = str2double(fgetl(IMU));%Gx
        EcaData(2) = str2double(fgetl(IMU));%Gy
        EcaData(3) = str2double(fgetl(IMU));%Gz
        EcaData(4) = str2double(fgetl(IMU));%Ax
        EcaData(5) = str2double(fgetl(IMU));%Ay
        EcaData(6) = str2double(fgetl(IMU));%Az
        EcaData(7) = str2double(fgetl(IMU));%Rotx
        EcaData(8) = str2double(fgetl(IMU));%Roty
        EcaData(9) = str2double(fgetl(IMU));%Rotz
        if (strcmp(typeData, 'Gyro'))           
            datax=EcaData(1);
            datay=EcaData(2);
            dataz=EcaData(3);
        elseif (strcmp(typeData, 'acelero'))
            datax=EcaData(4);
            datay=EcaData(5);
            dataz=EcaData(6);
        elseif (strcmp(typeData, 'Eulerangle'))
            datax=EcaData(7);
            datay=EcaData(8);
            dataz=EcaData(9);
        end
    end

  
end