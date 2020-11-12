%This script reads specified data and calculates error metrics
clear all
%User must be able to specify:
% - One of the algorithms {VO, MARTON}
% - Any comibation of datasets {20-04-09, 20-11-3-sim}
% - Any combination of occlusions {1,2,3}
% - Any combination of experiments {hl,hm,hh,mh,lh}


% I can try out some different metrics such as
% RMSE
% First order drift plus RMSE
% First order drift plus RMSE (and note of mean error)

% Regarding method
% Create a cell array
% loop through modes vector
% skip all modes 1 (angulation)
% 


% Regarding plots
% modes: {1,3,4}={azipe,VO,Marton}



% CHoose which configuration by setting the indexed in nmbrsX-vectors
directories={'20-04-09/','20-11-3-sim/'};  nmbr1=[1];
datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};nmbr2=[1,2,3,4];
algorithms={'VO','MARTON'};             nmbr3=1; %Only choose one
settings={'hl','hm','hh','mh','lh'};    nmbr4=[3,4,5];%1,2,3 or 3,4,5 
occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'}; nmbr5=[3];% 1 or 2 or 3?
basePath = '../data/';


G={};%Create cell array that is to be filled with data
runs=length(nmbr1)*length(nmbr2)*length(nmbr4)*length(nmbr5);
%Each fb sequence has one element in the cell arrays
%Each row contains sequences from a single log file
%[FBsequence 1, FBSequence2, ..., FBequence n1;
% FBSequence 1, FBSequence2, ..., FBSequence n2;
%                   ...
% FBSequence 1, FBSequence2, ... FBSequence nm]
%
X = {}; 
Y = {};
Z = {};
YAW = {};
T = {};

GTX = {};%Ground truth
GTY = {};%Ground truth
GTZ = {};%Ground truth
GTYAW = {};%Ground truth
GTT = {};%Should be equal to T


%DIMENSIONS: )inter-sequence, intra-sequence, intra data file)
counter=1;
%% Loop through data
for i=nmbr1
   for ii=nmbr2
gtPath = [basePath,directories{i},datasets{ii},'AZIPE_log.csv'];
[t_gt, x_gt, y_gt, z_gt, roll_gt, pitch_gt, yaw_gt, modes_gt]=getData(gtPath);
       for iii=nmbr4
           for iiii=nmbr5
path = [basePath,directories{i},datasets{ii},algorithms{nmbr3},'_',settings{iii},'_',occlusions{iiii},'_log.csv'];
[t, x, y, z, roll, pitch, yaw, modes]=getData(path);
% calculate errors
fbindexes=find(modes~=1)'; %Find all the indexes that does _not_ correspond to angulation. These are the relevant ones
steps = [1, diff(fbindexes)]; %If there is a step>1, the indexes are not adjescent and angulation must have been performed between
breakpoints = find(steps>1);%Find the steps. Each step correspond the first index in a Fallback sequence
% The breakpoints correspond to the first index in each sequence in
% fbindexes
%index fbindexes som korresponderar mot
%1 till breakpoitns(1)-1
%breakpoints(1) till breakpoints(2)-1
from = 1;
for j=1:length(breakpoints)
to = breakpoints(j)-1;
indexesOfFBSequencej = fbindexes(from:to);%These are the data indexes that we want to extract
%-ta ut data och spara i matris (from,to)
X{counter,j} = x(indexesOfFBSequencej);
Y{counter,j} = y(indexesOfFBSequencej);
Z{counter,j} = z(indexesOfFBSequencej);
YAW{counter,j} = yaw(indexesOfFBSequencej);
T{counter,j} = t(indexesOfFBSequencej);
% GT Data. 
% We have a single GT data file (with full framerate)
% Extract correct data points by matching timestamps
gtindexes = [];
for ti = T{counter,j}' %Must be a row vector
   in = find(t_gt==ti);
   gtindexes = [gtindexes,in];% Add every ground truth index in a vector
    if length(in)~=1
    error('Something wrong in GT index search')
    end
end
GTX{counter,j} = x_gt(gtindexes);
GTY{counter,j} = y_gt(gtindexes);
GTZ{counter,j} = z_gt(gtindexes);
GTYAW{counter,j} = yaw_gt(gtindexes);
GTT{counter,j} = t_gt(gtindexes);


from = breakpoints(j);%Start of next FB sequence
end
counter = counter+1;
           end
       end
   end
end



%At this point we have the complete matrices

%Get columns on by one
%Check isempty before calculating
index = 12;
figure
for i=1:runs
    if ~isempty(X{i,index})
        deltX=X{i,index}-GTX{i,index};
        deltY=Y{i,index}-GTY{i,index};
        
        plot(deltX,'.','color','k');
        hold on;
    end
end
title(algorithms{nmbr3})