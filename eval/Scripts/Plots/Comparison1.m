
%This script does 9 comparisons. 3 complexities vs 3 occlusion cases.
%Each comparison is further divided into x-y, z, and yaw
%This evaluation does not take sequence index into consideration - only
%total value

%directories={'20-04-09/','20-11-3-sim/'}; --nmbr1 = [1,2]
%datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};%nmbr2=[1,2,3,4];
%algorithms={'VO','MARTON'};            % nmbr3=2; %Only choose one
%settings={'hl','hm','hh','mm','mh','lm','lh'};    %nmbr4=[3];%1,2,3 or 3,4,5 
%occlusions={'AZ30FB15','AZ10FB20','AZ5FB40'};% nmbr5=[1,2,3];% 1 or 2 or 3?


nmbr3 = 2;          %algorithms={'VO','MARTON'}
nmbr4=[1,2,3];      %settings={'hl','hm','hh','mm','mh','lm','lh'};

nmbr1=[1];          %directories={'20-04-09/','20-11-3-sim/'};
nmbr2 = [1,2,3,4];  %datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};

nmbr5=[1,2,3];          %occlusions={'AZ30FB15','AZ10FB20','AZ5FB40'}
runs = length(nmbr1)*length(nmbr2)*length(nmbr4)*length(nmbr5);
basePath = '../data/';

% ResultMatrix
resultMatrix = zeros(10,3);

%% calc

%At this point we have the complete matrices
%Loop through all complexities and occlusions
for complexityIndex = 1:length(nmbr4)
    executionTime = [];
for occlusionIndex = 1:length(nmbr5)
    disp(['Getting data...']); %Only get data for the specified occlusion and the specified complexity setting Should be 4 rows (4 datasets, 1 directory, 1 occlusion, 1 complexity)
    [X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,T] = extractFBSequences(nmbr1,nmbr2,nmbr3,nmbr4(complexityIndex),nmbr5(occlusionIndex),basePath);

    %Loop through every FB sequence
    [run,seq] = size(T);%Run: number of data files, seq: number of fallback sections (note that this may be lower on some files so therefore must chekc isempty before reading data
    n = 0;
    xysqrdsum = 0;
    zsqrdsum = 0;
    yawsqrdsum = 0;
    for i=1:run
        for j=1:seq
            if isempty(T(i,j)) break; end% Jump to next run if we encounter an empty sequence in the current run. Dont think we will see that here though as we only have one complexity and dataset a time
            n = n + length(T{i,j});
            % horizontal RMSE
            %xysqrdsum = xysqrdsum +sum((X{i,j}-GTX{i,j}).^2 + (Y{i,j}-GTY{i,j}).^2);
            xysqrdsum = xysqrdsum +sum(((X{i,j}-GTX{i,j})./GTZ{i,j}).^2 + ((Y{i,j}-GTY{i,j})./GTZ{i,j}).^2);
            
            % Vertical RMSE - normed with ground truth Z
            % Relative error
            %zsqrdsum = zsqrdsum + sum((Z{i,j}-GTZ{i,j}).^2);
            zsqrdsum = zsqrdsum + sum((Z{i,j}./GTZ{i,j}-1).^2);
            % Yaw RMSE
            %Lite balett för att inte få +-2pi fel
                diffYAW = YAW{i,j}-GTYAW{i,j};
                toobigindexes = find(diffYAW>3);
                diffYAW(toobigindexes) = diffYAW(toobigindexes)-2*pi;
                toosmallindexes = find(diffYAW<-3);
                diffYAW(toosmallindexes) = diffYAW(toosmallindexes)+2*pi;
            yawsqrdsum = sum(diffYAW.^2);
            
            executionTime = [executionTime; EXECT{i,j}];
            

        end
    end
    
    
    % Edit to make error relative to Z???
    % Or in loop above just save delta values in loong vectors and do
    % caluclateions outside? maybe better?
    xyRMSE = sqrt(xysqrdsum/n);
    zRMSE = sqrt(zsqrdsum/n);
    yawRMSE = sqrt(yawsqrdsum/n);
    
    
    
    % Location of the calculated data in the big matrix
    tableCol = complexityIndex;
    tableRow = 3*(occlusionIndex-1); %+1 for xy, +2 for z, +3 for yaw
    
    resultMatrix(tableRow+1,tableCol) = xyRMSE*100;
    resultMatrix(tableRow+2,tableCol) = zRMSE*100;
    resultMatrix(tableRow+3,tableCol) = yawRMSE/(2*pi)*100;
    
end


resultMatrix(10,complexityIndex) = mean(executionTime);



end

disp(round(resultMatrix,3))
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




