%close all
clear all
d_est= csvread('../../outFile.csv',1,0);
%d_est= csvread('../VOAffine.csv',1,0);
%d_est= csvread('../AzAndVOAffine.csv',1,0);
%d_est= csvread('../VOAffine_noderot.csv',1,0);
%d_est= csvread('../VOAffine_noderot.csv',1,0);

    
    t_est = d_est(:,1)./1000;
    x_est = d_est(:,2);
    y_est = d_est(:,3);
    z_est = d_est(:,4);
    roll_est = d_est(:,5);
    pitch_est = d_est(:,6);
    yaw_est = d_est(:,7);

     figure
    subplot(3,1,1);
        plot(t_est,roll_est,'.');
        hold on
        plot(t_est,pitch_est,'.');
        title('Roll/pitch estimation');
        legend('Roll','Pitch');
        %axis([0,80,-2.5,2.5]);
    subplot(3,1,2);
        plot(t_est,x_est,'.');
        title('X direction');
    subplot(3,1,3);
        plot(t_est,y_est,'.');
        title('Y direction');

    figure
        plot(x_est,y_est,'.');
        xlabel('X dir [m]');
        ylabel('Y dir [m]');
        axis equal
    title('Position')
     