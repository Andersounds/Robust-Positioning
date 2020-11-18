
%This script does 9 comparisons. 3 complexities vs 3 occlusion cases.
%Each comparison is further divided into x-y, z, and yaw
%This evaluation does not take sequence index into consideration - only
%total value

%directories={'20-04-09/','20-11-3-sim/'}; --nmbr1 = [1,2]
%datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};%nmbr2=[1,2,3,4];
%algorithms={'VO','MARTON'};            % nmbr3=2; %Only choose one
%settings={'hl','hm','hh','mh','lh'};    %nmbr4=[3];%1,2,3 or 3,4,5 
%occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'};% nmbr5=[1,2,3];% 1 or 2 or 3?
disp('RUN FROM DIRECTORY eval/Scripts');
nmbr3 = 2;          %algorithms={'VO','MARTON'}
plotIdentifications = {'VO','MARTON'};

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




figure(nmbr3);
set(nmbr3,'Position',[1,1,850,400],'PaperUnits','centimeters','PaperSize',[29, 15]);
%Get all data for the specified directory
for directoryIndex = 1:length(nmbr1)
    directory = nmbr1(directoryIndex);
[X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,T] = extractFBSequences(directory,nmbr2,nmbr3,nmbr4,nmbr5,basePath);

xy_re = zeros(100,2); %One column for summation and one column for counter
z_re = zeros(size(xy_re));% Z_Relative Error
yaw_ae = zeros(size(xy_re));%YAW Absolute Error

%These vectors are to save all relative errors so that they can be shown as
%scatterplots
XY_RE = [];
Z_RE = [];
YAW_AE = [];





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
                for yi = 1:K
                   while (abs(diffYAW(yi)-2*pi) < abs(diffYAW(yi))) diffYAW(yi) = diffYAW(yi)-2*pi;end
                   while (abs(diffYAW(yi)+2*pi) < abs(diffYAW(yi))) diffYAW(yi) = diffYAW(yi)+2*pi;end  
                end
            YAW_AE_ij = abs(diffYAW);
        
        %Save errors for scatter plot
            indexes_ij = [1:K]';
            XY_RE = [XY_RE;[indexes_ij';XY_RE_ij']'];
            Z_RE = [Z_RE;[indexes_ij';Z_RE_ij']'];
            YAW_AE = [YAW_AE;[indexes_ij';YAW_AE_ij']'];
        
        
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
        
     
    %% Calculate trend and offset
    index = 1:length(xy_rrmse);
    
    pxy = polyfit(index',xy_rrmse,1);
    pz = polyfit(index',z_rrmse,1);
    pyaw = polyfit(index',yaw_rmse,1);
    
    resultMatrix(1+3*(directoryIndex-1),1:2) = pxy;
    resultMatrix(2+3*(directoryIndex-1),1:2) = pz;
    resultMatrix(3+3*(directoryIndex-1),1:2) = pyaw;
    disp('Make sure that trend and offset are in correct column')
    
    %% Scatterplot
        subplot(2,3,1+3*(directoryIndex-1))
        plot(XY_RE(:,1),XY_RE(:,2),'.','color',[0.2,0.2,0.2]);hold on
        subplot(2,3,2+3*(directoryIndex-1))
        plot(Z_RE(:,1),Z_RE(:,2),'.','color',[0.2,0.2,0.2]);hold on
        subplot(2,3,3+3*(directoryIndex-1))
        plot(YAW_AE(:,1),YAW_AE(:,2),'.','color',[0.2,0.2,0.2]);hold on
    
    %% Plot rrmse and rmse
    subplot(2,3,1+3*(directoryIndex-1))
        plot(xy_rrmse,'color','k','linewidth',2);
    subplot(2,3,2+3*(directoryIndex-1))
        plot(z_rrmse,'color','k','linewidth',2);
    subplot(2,3,3+3*(directoryIndex-1))
        plot(yaw_rmse,'color','k','linewidth',2);
    
    
end

% Edit some axis and text properties
algorithmTitles={'Visual Odometry','Polynomial Regression'};
    subplot(2,3,1)
    title('xy')
    subplot(2,3,2)
    title({[algorithmTitles{nmbr3}, ' - Measured dataset'],'z'})
    subplot(2,3,3)
    title('yaw')
    subplot(2,3,4)
    title('x,y')
    subplot(2,3,5)
    title({[algorithmTitles{nmbr3}, ' - Simulated dataset'],'z'})
    subplot(2,3,6)
    title('yaw')



if nmbr3 == 1
    %XY
    xyaxis = [0,40,0,0.3];
    zaxis = [0,40,0,0.2];
    yawaxis = [0,40,0,0.25];
elseif nmbr3 == 2
    xyaxis = [0,40,0,0.3];
    zaxis = [0,40,0,0.2];
    yawaxis = [0,40,0,0.5];
end

    subplot(2,3,1)
        axis(xyaxis)
        ylabel('RRMSE [-]')
        xlabel('Sequence index')
        legend('Relative error','RRMSE')
    subplot(2,3,4)
        axis(xyaxis)
        ylabel('RRMSE [-]')
        xlabel('Index after failure [-]')
        xlabel('Sequence index')
        legend('Relative error','RRMSE')
    subplot(2,3,2)
        axis(zaxis)
        ylabel('RRMSE [-]')
        xlabel('Sequence index')
        legend('Relative error','RRMSE')
    subplot(2,3,5)
        axis(zaxis)
        ylabel('RRMSE [-]')
        xlabel('Sequence index')
        legend('Relative error','RRMSE')
    subplot(2,3,3)
        axis(yawaxis)
        ylabel('RMSE [rad]')
        xlabel('Sequence index')
        legend('Absolute error','RMSE')
    subplot(2,3,6)
        axis(yawaxis)
        ylabel('RMSE [rad]')
        xlabel('Sequence index')
        legend('Absolute error','RMSE')

        
outpath = '../../../../../Documentation/Report_presentations/AndersonThesis/Texter/Results/Figures/';
%outpath = '../../../../../Documentation/Report_presentations/';
filetitle = ['Comparison3-indexederrors-',plotIdentifications{nmbr3}];
fulltitle = [outpath,filetitle,'.pdf'];        
        
   
%saveas(nmbr3,fulltitle)
print(nmbr3,'-dpdf','-bestfit',fulltitle)


disp(resultMatrix)
disp('Done')



