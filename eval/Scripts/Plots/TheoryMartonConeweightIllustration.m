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
aimode = 2;     legendStr{aimode} = 'aipe';
vomode = 3;     legendStr{vomode} = 'Visual Odometry';  %Visual odometry was used to estimate position (from last)
martmode = 4;   legendStr{martmode} = 'Polynomial regression';         % Marton was used to estimate position
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
basepath = '../data/illustrations/';
groundtruth = [basepath,'AZIPE_log.csv'];
withcone = [basepath,'MARTON_sim-23-marton-illustration.csv'];
withoutcone = [basepath,'MARTON_sim-23-marton-noCone-illustration.csv'];


plotTrue = 1;
disp('För att välja vilken att plotta, ändra plotCone mellan 0 och 1')
plotCone=1;
if plotCone==1
    figurenumbr = 10;
    figureTitle = 'With geometric constraint';
    d_est= csvread(withcone,1,0);
else
    figurenumbr = 11;
    figureTitle = 'Without geometric constraint';
    d_est= csvread(withoutcone,1,0);
end


%% Estimation File 1 and parameters
%Read file


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

d_ref= csvread(groundtruth,1,0);
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
 
        
        
     %% The plot that is saved  
        

     allowedModes = [4];
  
figure(figurenumbr);
clf(figurenumbr,'reset')
set(figurenumbr,'Position',[100,200,500,400],'PaperUnits','centimeters','PaperSize',[14, 14]);     
     
     

        if plotTrue
        plot(x_ref,y_ref,'Color',colors(1,:)); 
        %plot3(x_ref,y_ref,z_ref,'Color',colors(1,:),'Marker','.','LineStyle','None');
        %plot3(x_ref,y_ref,-z_ref,'Color',colors(1,:));
        hold on
        end
        % Plot est path in different colour depending on mode used
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            if length(find(allowedModes==i))==0
              continue
            end 
            plot(x_est(indeces),y_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            %plot3(x_est(indeces),y_est(indeces),-z_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            hold on      
        end
        

        xlabel('X dir [m]','FontSize',14);
        ylabel('Y dir [m]','FontSize',14);
        if plotCone==1
            legend('Ground truth','Polynomial regression');
        else
            legend('Ground truth','Polynomial regression');
        end
        axis([0,3.5,0,3])
    title(figureTitle,'Interpreter', 'none')
     set(gca,'FontSize',12)
     
     
    if plotCone==1
       filetitle = 'martonTheory-cone';
    else
       filetitle = 'martonTheory-Nocone';
    end    
     
outpath = '/Users/Fredrik/Google Drive/Kurser/Thesis/Documentation/Report_presentations/AndersonThesis/Texter/Theory/Figures/';
fulltitle = [outpath,filetitle,'.pdf'];        
print(figurenumbr,'-dpdf','-bestfit',fulltitle)