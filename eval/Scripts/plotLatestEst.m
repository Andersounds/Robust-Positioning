%close all
clear all
%% Estimation File and parameters
%d_est_path = '/Users/Fredrik/Datasets/20-04-09/';
%NUMBRS = flip([1,2,3,4,6,7,8,9,10,11,12,13,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29]);
%NUMBRS = [18,23,27,28]
NUMBRS = [27]
for NUM=NUMBRS
d_est_file = strcat('../data/20-11-3-sim/20-04-09-',num2str(NUM),'/MARTON_hh_AZ10FB20_log.csv');

d_est= csvread(d_est_file,1,0);
   
    t_est = d_est(:,1)./1000;
    x_est = d_est(:,2);
    y_est = d_est(:,3);
    z_est = d_est(:,4);
    roll_est = d_est(:,5);
    pitch_est = d_est(:,6);
    yaw_est = d_est(:,7);

     figure
    subplot(5,1,1);
        plot(t_est,roll_est,'.');
        hold on
        plot(t_est,pitch_est,'.');
        title('Roll/pitch estimation');
        legend('Roll','Pitch');
        %axis([0,80,-2.5,2.5]);
    subplot(5,1,2);
        plot(t_est,x_est,'.');
        title('X direction');
    subplot(5,1,3);
        plot(t_est,y_est,'.');
        title('Y direction');
    subplot(5,1,4);
        plot(t_est,-z_est,'.');
        title('Z direction');
    subplot(5,1,5);
        plot(t_est,yaw_est,'.');
        title('YAW');

    figure
        plot(x_est,y_est);
        xlabel('X dir [m]');
        ylabel('Y dir [m]');
        axis equal
    title(['Position. Dataset ',NUM])
  end