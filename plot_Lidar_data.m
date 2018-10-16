%%%
function []=plot_Lidar_data(measurement_data3)
measurement_data(:,1)=measurement_data3(:,1);
measurement_data(:,2)=measurement_data3(:,2);
figure(102)
hold on;plot(measurement_data(:,1),measurement_data(:,2),'+g');
color='g';
axis equal