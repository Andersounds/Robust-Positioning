%This script reads specified data and returns a cell arrays for x,y,z,yaw
%for the specified FB method as well as ground truth. It also returns
%execution times
%User must be able to specify:
% - One of the algorithms {VO, MARTON}
% - Any comibation of datasets {20-04-09, 20-11-3-sim}
% - Any combination of occlusions {1,2,3}
% - Any combination of experiments {'hl','hm','hh','mm','mh','lm','lh'};

function [X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,GTT] = extractFBSequences(nmbr1,nmbr2,nmbr3,nmbr4,nmbr5,basePath)
% CHoose which configuration by setting the indexed in nmbrsX-vectors
directories={'20-04-09/','20-11-3-sim/'};  %nmbr1=[1];
datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};%nmbr2=[1,2,3,4];
algorithms={'VO','MARTON'};            % nmbr3=2; %Only choose one
settings={'hl','hm','hh','mm','mh','lm','lh'};   %nmbr4=[3];%1,2,3 or 3,4,5 
occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'};% nmbr5=[1,2,3];% 1 or 2 or 3?
%basePath = '../data/';


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
EXECT = {};

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
[t, x, y, z, roll, pitch, yaw, modes, execTime]=getData(path);
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
EXECT{counter,j} = execTime(indexesOfFBSequencej);
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

end