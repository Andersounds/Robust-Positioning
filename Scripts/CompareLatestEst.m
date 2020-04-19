%close all
clear all
%%SETTINGS%%%%
basePath = '/Users/Fredrik/Datasets/Sim/20-04-19-1/';
plotTrue = 1;
%% Estimation File and parameters
%Read file
d_est_file = [basePath,'outFile.csv'];
d_est= csvread(d_est_file,1,0);
%Data columns
tcolE = 1;
xcolE = 2;
ycolE = 3;
zcolE = 4;
rollcolE = 5;
pitchcolE = 6;
yawcolE = 7;
modeCol = 9;
% Create data vectors 
% Extract modes
mode_est = d_est(:,modeCol);
C = unique(mode_est);%Modes that are logged

t_est = d_est(:,tcolE)./1000;
x_est = d_est(:,xcolE);
y_est = d_est(:,ycolE);
z_est = d_est(:,zcolE);
roll_est = d_est(:,rollcolE);
pitch_est = d_est(:,pitchcolE);
yaw_est = d_est(:,yawcolE);
mode_est = d_est(:,modeCol);



%% True path file and parameters
if plotTrue
disp('Reading true path file...');
%Read file
d_true_file = [basePath,'path-200419-1.csv'];
d_ref= csvread(d_true_file,1,0);
% data columns in file
tcolR = 1;
xcolR = 2;
ycolR = 3;
zcolR = 4;
rollcolR = 5;
pitchcolR = 6;
yawcolR = 7;
%Create data vectors
t_ref = d_ref(:,tcolR)./1000;
x_ref = d_ref(:,xcolR);
y_ref = d_ref(:,ycolR);
z_ref = d_ref(:,zcolR);
roll_ref = d_ref(:,rollcolR);
pitch_ref = d_ref(:,pitchcolR);
yaw_ref = d_ref(:,yawcolR);

end

%%Plot

     figure
    subplot(3,1,1);
        plot(t_est,roll_est,'.');
        hold on
        plot(t_est,pitch_est,'.');
        title('Roll/pitch');
        legend('Roll','Pitch');
        %axis([0,80,-2.5,2.5]);
    subplot(3,1,2);
        plot(t_est,x_est,'.');
        title('X direction');
    subplot(3,1,3);
        plot(t_est,y_est,'.');
        title('Y direction');

    figure
        if plotTrue
        plot(x_ref,y_ref,'.');
        hold on
        end
        % Plot est path in different colour depending on mode used
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
             plot(x_est(indeces),y_est(indeces),'.');
             hold on      
        end
        
        if plotTrue
             Legend = cell(length(C)+1,1);
             Legend{1} = 'Reference';
             for i=1:length(C)
                Legend{i+1} = num2str(C(i));
             end
             legend(Legend);
        else
             %legendCell = cellstr(num2str(C', 'N=%-d'));
             %legend(legendCell);
             Legend = cell(length(C),1);
             for i=1:length(C)
                Legend{i} = num2str(C(i));
             end
             legend(Legend);
        end
        xlabel('X dir [m]');
        ylabel('Y dir [m]');
        axis equal
    title('Position')
     