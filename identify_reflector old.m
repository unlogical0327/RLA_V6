%% Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
function [detected_ID,detected_reflector]=identify_reflector(amp_thres,angle_delta,distance_delta,Lidar_Table,Lidar_data)
detected_ID=0;
iii=1;
loss=-30;    % loss in dB, need the calibration
%amp=2096;    % max amplitude from calibration
reflector_diameter=200;
amp_thres
detected_reflector=0;
for ii=2:length(Lidar_data)

    if Lidar_data(3,ii)>=amp_thres
        Lidar_data(3,ii);
            angle_delta=reflector_diameter/Lidar_data(2,ii)/pi*180
        if (abs(Lidar_data(2,ii)-Lidar_data(2,ii-1))<distance_delta) && (abs(Lidar_data(1,ii)-Lidar_data(1,ii-1))<angle_delta)
             % check if the detected point is from the same reflector
                1
                detected_ID(iii)=iii;
                if detected_reflector(detected_ID(iii),1)==0
                detected_reflector(detected_ID(iii),1)=Lidar_Table(ii,1);
                detected_reflector(detected_ID(iii),2)=Lidar_Table(ii,2);
                else
                detected_reflector(detected_ID(iii),1)=(Lidar_Table(ii,1)+detected_reflector(detected_ID(iii),1))/2;
                detected_reflector(detected_ID(iii),2)=(Lidar_Table(ii,2)+detected_reflector(detected_ID(iii),2))/2;
                end
            else %(abs(Lidar_data(1,ii)-Lidar_data(1,ii-1))<angle_delta)
                2
                iii=iii+1;
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1)=Lidar_Table(ii,1);
                detected_reflector(detected_ID(iii),2)=Lidar_Table(ii,2);
                disp('Detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID(iii)));
                
         end
    end
end
figure(104);plot(detected_reflector(:,1),detected_reflector(:,2),'+')
if (length(detected_ID)<=1 && detected_ID(1)==0)
    detected_reflector=0;
    disp('No reflector Detected!!!');
end