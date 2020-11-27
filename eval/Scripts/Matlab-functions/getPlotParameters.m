function [legendStr, colors] = getPlotParameters()


%% Plot settings - legends, colors
%Define Legend strings and pair them with return modes
legendStr = {}; %Create a struct to hold legend strings
%Return flags for what positioning algorithm was used
azmode = 1;     legendStr{azmode} = 'Angulation';       %Azipe angulation was used to estimate position
aimode = 2;     legendStr{aimode} = 'aipe';             % NOT USED
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
% Mode 3: VO
% Mode 4: Polynomial Regression
colors = [0, 0.4470, 0.7410;      % blue (Reference)
        0.3010, 0.7450, 0.9330;   % light blue (Azipe)
        0.9290, 0.6940, 0.1250;   % yellow
        0.8500, 0.3250, 0.0980;   % orange (VO)
        0.4940, 0.1840, 0.5560;   % purple (Marton)
        0.4660, 0.6740, 0.1880;   % Green
        0.6350, 0.0780, 0.1840;   % dark red
        0.25, 0.25, 0.25;% Very dark green? or gray
        0, 0.5, 0; % lighter green
    ];


end