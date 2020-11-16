
%This script does 9 comparisons. 3 complexities vs 3 occlusion cases.
%Each comparison is further divided into x-y, z, and yaw
%This evaluation does not take sequence index into consideration - only
%total value

%directories={'20-04-09/','20-11-3-sim/'}; --nmbr1 = [1,2]
%datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};%nmbr2=[1,2,3,4];
%algorithms={'VO','MARTON'};            % nmbr3=2; %Only choose one
%settings={'hl','hm','hh','mh','lh'};    %nmbr4=[3];%1,2,3 or 3,4,5 
%occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'};% nmbr5=[1,2,3];% 1 or 2 or 3?

nmbr3 = 1;          %algorithms={'VO','MARTON'}

if nmbr3 == 1
    nmbr4=[2];      %settings={'hl','hm','hh','mm','mh','lm','lh'};  
elseif nmbr3 == 2
    nmbr4=[2];      %settings={'hl','hm','hh','mm','mh','lm','lh'};
else
    disp('ONLY CHOOSE ONE ALGORITHM: VO (1) or MARTON (2)') 
end


nmbr1=[1,2];          %directories={'20-04-09/','20-11-3-sim/'};
nmbr2 = [1,2,3,4];  %datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};

nmbr5=[1,2,3];          %occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'}
runs = length(nmbr1)*length(nmbr2)*length(nmbr4)*length(nmbr5);
basePath = '../data/';

% ResultMatrix
resultMatrix = zeros(6,3);

%% c

xy_re = zeros(100,2); %One column for summation and one column for counter
z_re = zeros(size(xy_re));% Z_Relative Error
yaw_ae = zeros(size(xy_re));%YAW Absolute Error
figure
%Get all data for the specified directory
for directory = nmbr1
[X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,T] = extractFBSequences(directory,nmbr2,nmbr3,nmbr4,nmbr5,basePath);

    %Loop through every FB sequence
    [run,seq] = size(T);%Run: number of data files, seq: number of fallback sections (note that this may be lower on some files so therefore must chekc isempty before reading data
    for i=1:run
        for j=1:seq
            if isempty(T(i,j)) break; end% Jump to next run if we encounter an empty sequence in the current run. Dont think we will see that here though as we only have one complexity and dataset a time
            K = length(T{i,j});
            
       %Calculate element wise Relative Error
            XY_RE_ij = abs((X{i,j}-GTX{i,j})./GTZ{i,j}) + abs((Y{i,j}-GTY{i,j})./GTZ{i,j});
            Z_RE_ij = abs(Z{i,j}./GTZ{i,j}-1);
       %For yaw calculate Absolute Error
                %Fixar yaw för att inte få +- 2pi fel   
            diffYAW = YAW{i,j}-GTYAW{i,j};
                for yi = 1:length(diffYAW)
                   while (abs(diffYAW(yi)-2*pi) < abs(diffYAW(yi))) diffYAW(yi) = diffYAW(yi)-2*pi;end
                   while (abs(diffYAW(yi)+2*pi) < abs(diffYAW(yi))) diffYAW(yi) = diffYAW(yi)+2*pi;end  
                end
            YAW_AE_ij = abs(diffYAW);
        % Plot as scatter plots
            subplot(1,3,1)
            plot(XY_RE_ij,'.','color',[0.2,0.2,0.2]);hold on
            subplot(1,3,2)
            plot(Z_RE_ij,'.','color',[0.2,0.2,0.2]);hold on
            subplot(1,3,3)
            plot(YAW_AE_ij,'.','color',[0.2,0.2,0.2]);hold on
            
        %Add squared elements to sum
            xy_re(1:K,1) = xy_re(1:K,1) + XY_RE_ij.^2;   %Element wise sum of squares
            xy_re(1:K,2) = xy_re(1:K,2) + 1;             %increase n by one for these indexes
            z_re(1:K,1)  = z_re(1:K,1) + Z_RE_ij.^2;
            z_re(1:K,2)  = z_re(1:K,2) + 1;           
            yaw_ae(1:K,1) = yaw_ae(1:K,1) + YAW_AE_ij.^2;
            yaw_ae(1:K,2) = yaw_ae(1:K,2) + 1;
               
       
            
        end
    end
    %Calculate RRMSE and RMSE values for xyz and yaw, respectively
    nonZeroIndexes = length(find(xy_re(:,2)~=0));


    xy_rrmse = sqrt(xy_re(1:nonZeroIndexes,1)./xy_re(1:nonZeroIndexes,2));
    z_rrmse = sqrt(z_re(1:nonZeroIndexes,1)./z_re(1:nonZeroIndexes,2));
    yaw_rmse = sqrt(yaw_ae(1:nonZeroIndexes,1)./yaw_ae(1:nonZeroIndexes,2));
        
           
    subplot(1,3,1)
        plot(xy_rrmse,'color','r');
    subplot(1,3,2)
        plot(z_rrmse,'color','r');
    subplot(1,3,3)
        plot(yaw_rmse,'color','r');
    
    
end



disp(round(resultMatrix,2))
disp('Done')






% 
% 
% 
% 
% %Get columns on by one
% %Check isempty before calculating
% figure
% for i=1:runs %outer loop: every run (every file)
%     for j=1:length(T(i,:)) %inner loop: every fallback sequence
%     if isempty(T(i,j)) break; end %Need to check this because if one row is longer the others are filled with empty cells;   
%    
%     errXY = ((X{i,j}-GTX{i,j}).^2 + (Y{i,j}-GTY{i,j}).^2).^(1/2);%RMSE
%     errZ=abs(Z{i,j}-GTZ{i,j});
%     %Lite balett för att inte få +-2pi fel
%     diffYAW = YAW{i,j}-GTYAW{i,j};
%     toobigindexes = find(diffYAW>3);
%     diffYAW(toobigindexes) = diffYAW(toobigindexes)-2*pi;
%     toosmallindexes = find(diffYAW<-3);
%     diffYAW(toosmallindexes) = diffYAW(toosmallindexes)+2*pi;
%     errYAW = abs(diffYAW);
%     
%     subplot(2,3,1)
%     %plot(errXY);hold on;
%       plot(errXY,'color',[0.95,0.95,0.95]);hold on;
%       plot(errXY,'.','color','k');hold on;
%       axis([0,40,0,0.3]);
%       xlabel('Fallback sequence index')
%       ylabel('Error [m]')
%       title('Visual Odometry - Horizontal RMSE')
%     subplot(2,3,2)
%     %plot(errZ);hold on;
%       plot(errZ,'color',[0.95,0.95,0.95]);hold on;
%       plot(errZ,'.','color','k');hold on;
%       axis([0,40,0,0.3])
%       xlabel('Fallback sequence index')
%       ylabel('Error [m]')
%       title('Visual Odometry - Vertical')
%     subplot(2,3,3)
%     %plot(errYAW);hold on;
%       plot(errYAW,'color',[0.95,0.95,0.95]);hold on;
%       plot(errYAW,'.','color','k');hold on;
%       axis([0,40,0,0.3]);
%       xlabel('Fallback sequence index')
%       ylabel('Error [rad]')
%       title('Visual Odometry - YAW')
%     end
% end
% 
% 
% 
% 
% 




