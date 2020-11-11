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
directories={'20-04-09/','20-11-3-sim/'};  nmbr1=[2];
datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};nmbr2=[1,2,3,4];
algorithms={'VO','MARTON'};             nmbr3=1; %Only choose one
settings={'hl','hm','hh','mh','lh'};    nmbr4=[1,2,3];%1,2,3 or 3,4,5 
occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'}; nmbr5=[3];% 1 or 2 or 3?
basePath = '../data/';


G={};%Create cell array that is to be filled with data
runs=length(nmbr1)*length(nmbr2)*length(nmbr4)*length(nmbr5);
X = zeros(100,100,runs); %Must be able to contain all data points. (rows>number of FB sequences, cols>length of each FB sequence)
Y = zeros(size(G));
Z = zeros(size(G));
YAW = zeros(size(G));
MASK = zeros(size(G));
GTX = zeros(size(G));%Ground truth
GTY = zeros(size(G));%Ground truth
GTZ = zeros(size(G));%Ground truth
GTYAW = zeros(size(G));%Ground truth

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
sequenceLength = length(indexesOfFBSequencej);

%-ta ut data och spara i matris (from,to)
X(1:sequenceLength,j,counter) = x(indexesOfFBSequencej);
Y(1:sequenceLength,j,counter) = y(indexesOfFBSequencej);
Z(1:sequenceLength,j,counter) = z(indexesOfFBSequencej);
YAW(1:sequenceLength,j,counter) = yaw(indexesOfFBSequencej);
MASK(1:sequenceLength,j,counter)=1;
%DIMENSIONS: )inter-sequence, intra-sequence, intra data file)

from = breakpoints(j);%Start of next FB sequence
end
counter = counter+1;
           end
       end
   end
end



%At this point we have the complete matrices