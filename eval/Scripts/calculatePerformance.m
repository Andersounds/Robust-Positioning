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
% Collect vector pairs with est and ground truth
%   -intra vector pairs may have different lengths (for different
%   occlusions or failed estimations)
% Use only pairs that are preceeded by azipe (maybe accept one or two
% failed FB)
% Match timestamps to get GT data fro full framerate azipe

% Regarding plots
% 



% CHoose which configuration by setting the indexed in nmbrsX-vectors
directories={'20-04-09/','20-11-3-sim/'};  nmbr1=[2];
datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};nmbr2=[1,2,3,4];
algorithms={'VO','MARTON'};             nmbr3=[1];
settings={'hl','hm','hh','mh','lh'};    nmbr4=[1,2,3];%1,2,3 or 3,4,5 
occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'}; nmbr5=[1];% 1 or 2 or 3?
basePath = '../data/';




%% Loop through data
for i=nmbr1
   for ii=nmbr2
       for iii=nmbr3
           for iiii=nmbr4
               for iiiii=nmbr5
path = [basePath,directories{i},datasets{ii},algorithms{iii},'_',settings{iiii},'_',occlusions{iiiii},'_log.csv'];
[t, x, y, z, roll, pitch, yaw, modes]=getData(path);
               end
           end
       end
   end
end