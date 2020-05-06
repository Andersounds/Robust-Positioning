%close all
clear all

%colors = [0.8500 0.3250 0.0980;
%        0.9290 0.6940 0.1250;
%        0 0.4470 0.7410;
%        0.4660 0.6740 0.1880;
%        0.6350 0.0780 0.1840;
%        0 0 0];
colors = [0, 0.4470, 0.7410;
        0.8500, 0.3250, 0.0980;
        0.9290, 0.6940, 0.1250;
        0.4940, 0.1840, 0.5560;
        0.4660, 0.6740, 0.1880;
        0.3010, 0.7450, 0.9330;
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
%d_est_str = 'MARTON_tilt';
%d_est_str = 'VO_tilt';
%d_est_str = 'AZIPE_tilt';
d_est_str = 'AZIPE_filt_tilt';
%d_est_str = 'MARTON_notilt';
%d_est_str = 'VO_notilt';


%%%%  TRUE FILE BASE NAME  %%%%
%d_true_file = [basePath,'AZIPE_tilt.csv'];
d_true_file = [basePath,'AZIPE_notilt.csv'];


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
        counter = 2;
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(counter,length(colors))+1;
            plot(t_est(indeces),x_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            counter = counter+1;
            hold on      
        end
        title('X direction');
    subplot(5,1,3);
        if plotTrue
            plot(t_ref,y_ref,'Color',colors(1,:));
            hold on
        end   
       %%% Plot in different colors depending on method   
        counter = 2;
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(counter,length(colors))+1;
            plot(t_est(indeces),y_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            counter = counter+1;
            hold on      
        end
        title('Y direction');
        subplot(5,1,4);
        if plotTrue
            plot(t_ref,z_ref,'Color',colors(1,:));
            hold on
        end   
       %%% Plot in different colors depending on method   
        counter = 2;
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(counter,length(colors))+1;
            plot(t_est(indeces),z_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            counter = counter+1;
            hold on      
        end
        title('Z direction');
    subplot(5,1,5);
        if plotTrue
            plot(t_ref,yaw_ref,'Color',colors(1,:));
            hold on
        end
        %%% Plot in different colors depending on method   
        counter = 2;
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(counter,length(colors))+1;
            plot(t_est(indeces),yaw_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            counter = counter+1;
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
        counter = 2;
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(counter,length(colors))+1;
            %plot(x_est(indeces),y_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            plot3(x_est(indeces),y_est(indeces),z_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            counter = counter+1;
            hold on      
        end
        
        if plotTrue
             Legend = cell(length(C)+1,1);
             Legend{1} = 'Reference';
             for i=1:length(C)
                Legend{i+1} = num2str(C(i));
             end
             legend(Legend);
             grid on
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
    title(d_est_str,'Interpreter', 'none')
     