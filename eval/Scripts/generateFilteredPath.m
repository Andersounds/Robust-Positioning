% This script reads a path, filters it, and outputs it again.
% To be used to generate a smooth path that can be used as simulation
% input


NUM = 28;
BASE = '/Users/Fredrik/Google Drive/Kurser/Thesis/Evaluation/';
infile_path = strcat(BASE,'20-04-09/20-04-09-',num2str(NUM),'/AZIPE_outFile.csv');
output_path_filtered = strcat(BASE,'20-11-3-sim/20-04-09-',num2str(NUM),'/GT.csv');
file= csvread(infile_path,1,0);
    t = file(:,1);
    x = file(:,2);
    y = file(:,3);
    z = file(:,4);
    roll = file(:,5);
    pitch = file(:,6);
    yaw = file(:,7);
    
    
   %% Process data.
   windowSize = 15; 
   b = (1/windowSize)*ones(1,windowSize);
   a = 1;
   z_filt = filter(b,a,z);
   z_filt(1:windowSize) = z_filt(windowSize+1);
  % filter z at least. maybe something more?
    
  x = x + 0.5;
  y = y + 0.5;
  dist = abs(z_filt./(cos(roll).*cos(pitch)));
  
    %% Write
total = [t';x';y';z_filt';roll';pitch';yaw';dist']';% Write to csv file
    
csvwrite(output_path_filtered,total,1,0) %Start writing at row 1, col 0 (0-indexed)


% t [s], x [m], y [m], z [m], roll [rad], pitch [rad], yaw [rad], dist [m]
%Plot data
     figure
    subplot(5,1,1);
        plot(t,roll,'.');
        hold on
        plot(t,pitch,'.');
        title('Roll/pitch');
        legend('Roll','Pitch');
        %axis([0,80,-2.5,2.5]);
    subplot(5,1,2);
        plot(t,x,'.');
        title('X direction');
    subplot(5,1,3);
        plot(t,y,'.');
        title('Y direction');
    subplot(5,1,4);
        plot(t,-z,'.');
        title('Z direction');
        hold on
        plot(t,-z_filt,'color','r')
        hold on
        plot(t,dist,'color','k')
    subplot(5,1,5);
        plot(t,yaw,'.');
        title('YAW');