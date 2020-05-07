% This script is to be used to plot comparisions of paths 
% in a consistent manner, with consistent colors and scales
% Save experiments in separate folder. Put generated paths there 
% For each figure in the report, save a plot script somehow 
%
%

%close all
clear all



%Define Legend strings and pair them with return modes
legendStr = {}; %Create a struct to hold legend strings
%Return flags for what positioning algorithm was used
azmode = 1;     legendStr{azmode} = 'Angulation';       %Azipe angulation was used to estimate position
vomode = 2;     legendStr{vomode} = 'Visual Odometry';  %Visual odometry was used to estimate position (from last)
martmode = 3;   legendStr{martmode} = 'Marton';         % Marton was used to estimate position
%Error codes are above 10
azFailed = 10;  legendStr{azFailed} = ' ';              %If only azipe is used and estimation fails
%VO error codes
voInertia = 20; legendStr{voInertia} = ' ';             %Visual odometry failed, assumed rotational speed kept
voproj = 21;    legendStr{voproj} = ' ';                %Visual odometry successful and estimation is projected onto v_tilde NOT USED
%Marton error codes
martfailed = 30;legendStr{martfailed} = ' ';
martold = 31;   legendStr{martold} = ' ';
marterr = 32;   legendStr{marterr} = ' ';




%First row is used by reference. next rows are for other codes
% Return code i uses color specified on row i+1
colors = [0, 0.4470, 0.7410;      % Reference
        0.3010, 0.7450, 0.9330;   % Azipe
        0.8500, 0.3250, 0.0980;   % VO
        0.9290, 0.6940, 0.1250;   % Marton
        0.4940, 0.1840, 0.5560;  
        0.4660, 0.6740, 0.1880;
        0.6350, 0.0780, 0.1840;
        0.25, 0.25, 0.25;
        0, 0.5, 0;
    ];
%%%%  BASEPATH  %%%%
%basePath = '/Users/Fredrik/Datasets/Sim/20-04-27-fullyaw/';
%basePath = '/Users/Fredrik/Datasets/Sim/20-04-23-1/';
basePath = '/Users/Fredrik/Datasets/20-04-09/20-04-09-22/';
%%%%  ESTIMATION FILE BASE NAME  %%%%
%d_est_str = 'AIPE';
%d_est_str = 'MARTON_out';
%d_est_str = 'MARTON_noweight';
d_est_str = 'VO_out';
%d_est_str = 'AZIPE_out';

%d_est_str = 'MARTON_notilt';
%d_est_str = 'VO_notilt';


%%%%  TRUE FILE BASE NAME  %%%%
d_true_file = [basePath,'AZIPE_out.csv'];
%d_true_file = [basePath,'AZIPE_notilt.csv'];


d_est_file = [basePath,d_est_str,'.csv'];

plotTrue = 1;
%% Estimation File and parameters
%Read file

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
     
    subplot(5,1,1);
        plot(t_est,roll_est,'.');
        hold on
        plot(t_est,pitch_est,'.');
        title('Roll/pitch');
        legend('Roll','Pitch');
        %axis([0,80,-2.5,2.5]);
    subplot(5,1,2);
        if plotTrue
            plot(t_ref,x_ref,'Color',colors(1,:));
            hold on
        end
  %%% Plot in different colors depending on method   

        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            plot(t_est(indeces),x_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));

            hold on      
        end
        title('X direction');
    subplot(5,1,3);
        if plotTrue
            plot(t_ref,y_ref,'Color',colors(1,:));
            hold on
        end   
       %%% Plot in different colors depending on method   

        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            plot(t_est(indeces),y_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            hold on      
        end
        title('Y direction');
        subplot(5,1,4);
        if plotTrue
            plot(t_ref,z_ref,'Color',colors(1,:));
            hold on
        end   
       %%% Plot in different colors depending on method   
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            plot(t_est(indeces),z_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            hold on      
        end
        title('Z direction');
    subplot(5,1,5);
        if plotTrue
            plot(t_ref,yaw_ref,'Color',colors(1,:));
            hold on
        end
        %%% Plot in different colors depending on method   
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            plot(t_est(indeces),yaw_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            hold on      
        end
        title('Yaw');

    figure
        if plotTrue
        %plot(x_ref,y_ref,'Color',colors(1,:),'Marker','.','LineStyle','None'); 
        %plot3(x_ref,y_ref,z_ref,'Color',colors(1,:),'Marker','.','LineStyle','None');
        plot3(x_ref,y_ref,z_ref,'Color',colors(1,:));
        hold on
        end
        % Plot est path in different colour depending on mode used
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            %plot(x_est(indeces),y_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            plot3(x_est(indeces),y_est(indeces),z_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            hold on      
        end
        
        if plotTrue
             Legend = cell(length(C)+1,1);
             Legend{1} = 'Reference';
             for i=1:length(C)
                  Legend{i+1} = legendStr{C(i)};
             end
             legend(Legend);
             grid on
        else
             %legendCell = cellstr(num2str(C', 'N=%-d'));
             %legend(legendCell);
             Legend = cell(length(C),1);
             if C(i) <= 3
                 Legend{i} = legendStr{C(i)};
             end
             legend(Legend);
        end
        xlabel('X dir [m]');
        ylabel('Y dir [m]');
        axis equal
    title(d_est_str,'Interpreter', 'none')
     