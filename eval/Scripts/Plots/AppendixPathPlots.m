clear all
%This could be a script that reads all occlusion cases and plots data
%as a 3x3 subplots where the columns are occlusioncases and the rows are x-y 2d plot, z, yaw
% At top of plot i have to choose:
% - marton or VO
% - sim or collected
% - settings hl,hm,hh,mh,lh
%Possibly choose if xy is 2x1d or 1x2d
%close all
clear all
algorithms={'VO','MARTON'};             nmbr1=1; %CHANGE AMGORITHM HERE
directories={'20-04-09/','20-11-3-sim/'};  nmbr2=1;
datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};nmbr3=4;%CHANGE SET HERE
settings={'hl','hm','hh','mm','mh','lm','lh'};    nmbr4=2;
occlusions={'AZ30FB15','AZ10FB20','AZ5FB40'}; 
plotTrue = 1;


occlusionTitles = {'Transient','Repeated','Static'};% For plot titles
dirTitles = {'20-04-09','20-11-3-sim'};
datasetTitles = {'18','23','27','28'};
figure(nmbr1);
clf(nmbr1,'reset')
set(nmbr1,'Position',[1,1,700,500],'PaperUnits','centimeters','PaperSize',[28, 21]);


if nmbr1==1
    outPath = '/Users/Fredrik/Google Drive/Kurser/Thesis/Documentation/Report_presentations/AndersonThesis/Texter/Appendix/Allpaths_VO/Figures/';    
elseif nmbr1==2
    outPath = '/Users/Fredrik/Google Drive/Kurser/Thesis/Documentation/Report_presentations/AndersonThesis/Texter/Appendix/Allpaths_Marton/Figures/';
end
fileTitle = ['Paths-Allocclusions-',algorithms{nmbr1},'-',dirTitles{nmbr2},'-',datasetTitles{nmbr3},'-',settings{nmbr4}];
algorithmTitles={'Visual Odometry','Polynomial Regression'};
%%%%  BASEPATH  %%%%
basePath = '../data/';
dir_str = directories{nmbr2};
nmbrs_str = datasets{nmbr3};
%%%%  ESTIMATION FILE BASE NAME  %%%%
mode_str = algorithms{nmbr1};
exp_str = settings{nmbr4};

%% Plot settings - legends, colors
[legendStr, colors] = getPlotParameters();

%% Plot

for occlusionindex=1:3
%% Estimation File and parameters
% Read data
d_est_file = [mode_str,'_',exp_str,'_',occlusions{occlusionindex},'_log.csv'];
d_est_file_full = [basePath,dir_str,nmbrs_str,d_est_file];
[t_est,x_est,y_est,z_est,roll_est,pitch_est,yaw_est,mode_est]=getData(d_est_file_full);
C = unique(mode_est);%Modes that are logged
%% True path file and parameters
if plotTrue
disp('Reading true path file...');
%% Read data
d_true_file_full = [basePath,dir_str,nmbrs_str,'AZIPE_log.csv'];
%Create data vectors
[t_ref, x_ref, y_ref, z_ref, roll_ref, pitch_ref, yaw_ref, modes_azipe]=getData(d_true_file_full);

end

%% X-Y Plot
subplot(3,3,occlusionindex);
plottitle={[''],['Occlusion: ',occlusionTitles{occlusionindex}],['X-Y path']};
if(occlusionindex==2)
plottitle{1}=['Algorithm: ', algorithmTitles{nmbr1}];

end

if plotTrue
   plot(x_ref,y_ref,'Color',colors(1,:));
   hold on
end
  % Plot est path in different colour depending on mode used
for i=C' %Must be row vector to loop through it like this
    indeces = find(mode_est==i);%Find indices where algorithm used mode i
	colorindex = mod(i,length(colors))+1;
    if(plotTrue && (i~=1))
    plot(x_est(indeces),y_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
    hold on 
    end
end
title(plottitle);
xlabel('[m]')
ylabel('[m]')
%% Z-plot
    subplot(3,3,occlusionindex+3);
    plottitle=['Z path'];
        if plotTrue
            plot(t_ref,-z_ref,'Color',colors(1,:));
            hold on
        end
  %%% Plot in different colors depending on method   

        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            if(plotTrue && (i~=1))
            plot(t_est(indeces),-z_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            hold on   
            end
        end
        title(plottitle);
        xlabel('Time [s]')
        ylabel('Height [m]')
%% Yawplot        
    subplot(3,3,occlusionindex+6);
    plottitle=['Yaw angle'];
        if plotTrue
            plot(t_ref,yaw_ref,'Color',colors(1,:));
            hold on
        end
        %%% Plot in different colors depending on method   
        for i=C' %Must be row vector to loop through it like this
            indeces = find(mode_est==i);%Find indices where algorithm used mode i
            colorindex = mod(i,length(colors))+1;
            if(plotTrue && (i~=1))
            plot(t_est(indeces),yaw_est(indeces),'Marker','.','LineStyle','None','Color',colors(colorindex,:));
            hold on  
            end
        end
      title(plottitle);
      xlabel('Time [s]')
      ylabel('Yaw [rad]')
    
end


Lgnd = legend('Ground truth',algorithmTitles{nmbr1});
Lgnd.Position(1) = 0.75;
Lgnd.Position(2) = 0;


fulltitle=[outPath,fileTitle,'.pdf'];

print(nmbr1,'-dpdf','-bestfit',fulltitle);