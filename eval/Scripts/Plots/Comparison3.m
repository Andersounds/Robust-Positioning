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
nmbr3 = 1;          %algorithms={'VO','MARTON'}

plotIdentifications = {'VO','MARTON'};

%Here the best performing complexity from previos comparisons shall be
%chosen.
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
clf(nmbr3,'reset')
set(nmbr3,'Position',[1,1,850,400],'PaperUnits','centimeters','PaperSize',[29, 15]);
%Get all data for the specified directory
for directoryIndex = 1:length(nmbr1)
    directory = nmbr1(directoryIndex);
[X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,T] = extractFBSequences(directory,nmbr2,nmbr3,nmbr4,nmbr5,basePath);

xy_re_sqrd = zeros(100,2); %One column for summation and one column for counter
z_re_sqrd = zeros(size(xy_re_sqrd));% Z_Relative Error
yaw_ae_sqrd = zeros(size(xy_re_sqrd));%YAW Absolute Error

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
            disp('Scaling relative errors with 100')
       %Calculate element wise Relative Error
            XY_RE_ij = 100*sqrt((X{i,j}-GTX{i,j}).^2 + (Y{i,j}-GTY{i,j}).^2)./abs(GTZ{i,j});
            %XY_RE_ij = abs((X{i,j}-GTX{i,j})./GTZ{i,j}) + abs((Y{i,j}-GTY{i,j})./GTZ{i,j});
            Z_RE_ij = 100*abs(Z{i,j}./GTZ{i,j}-1);
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
            xy_re_sqrd(1:K,1) = xy_re_sqrd(1:K,1) + XY_RE_ij.^2;   %Element wise sum of squares
            xy_re_sqrd(1:K,2) = xy_re_sqrd(1:K,2) + 1;             %increase n by one for these indexes
            z_re_sqrd(1:K,1)  = z_re_sqrd(1:K,1) + Z_RE_ij.^2;
            z_re_sqrd(1:K,2)  = z_re_sqrd(1:K,2) + 1;           
            yaw_ae_sqrd(1:K,1) = yaw_ae_sqrd(1:K,1) + YAW_AE_ij.^2;
            yaw_ae_sqrd(1:K,2) = yaw_ae_sqrd(1:K,2) + 1;
            
        end
    end
    %Calculate RRMSE and RMSE values for xyz and yaw, respectively
    % for each index separately
    nonZeroIndexes = length(find(xy_re_sqrd(:,2)~=0));

    
    xy_rrmse =sqrt(xy_re_sqrd(1:nonZeroIndexes,1)./xy_re_sqrd(1:nonZeroIndexes,2));
    z_rrmse =sqrt(z_re_sqrd(1:nonZeroIndexes,1)./z_re_sqrd(1:nonZeroIndexes,2));
    yaw_rmse =sqrt(yaw_ae_sqrd(1:nonZeroIndexes,1)./yaw_ae_sqrd(1:nonZeroIndexes,2));
    
    %% Calculate total RRMSE and RMSE for every error point for all occlusion cases. 
    %    Separate collected and sim, of course
   % disp('Caclulate single RRMSE and RMSE values for last column')
     
 %Below are total rrmse and rmse values, considering either collected or
 %simulated dataset, but all indexes of all occlusion cases.
     n = sum(xy_re_sqrd(:,2));
     xy_rrmse_tot =sqrt(sum(xy_re_sqrd(:,1))/n);
     z_rrmse_tot =sqrt(sum(z_re_sqrd(:,1))/n);
     yaw_rmse_tot =sqrt(sum(yaw_ae_sqrd(:,1))/n);
     
     resultMatrix(1+3*(directoryIndex-1),3) = xy_rrmse_tot;
     resultMatrix(2+3*(directoryIndex-1),3) = z_rrmse_tot;
     resultMatrix(3+3*(directoryIndex-1),3) = yaw_rmse_tot;
     
     
    %% Calculate trend and offset
    index = 1:length(xy_rrmse);
    
    pxy = polyfit(index',xy_rrmse,1);
    pz = polyfit(index',z_rrmse,1);
    pyaw = polyfit(index',yaw_rmse,1);
    
    resultMatrix(1+3*(directoryIndex-1),1:2) = pxy;
    resultMatrix(2+3*(directoryIndex-1),1:2) = pz;
    resultMatrix(3+3*(directoryIndex-1),1:2) = pyaw;
    %1:slope
    %2:offset
    
    %% Scatterplot
        subplot(2,3,1+3*(directoryIndex-1))
        plot(XY_RE(:,1),XY_RE(:,2),'.','color',[0.2,0.2,0.2]);hold on
        subplot(2,3,2+3*(directoryIndex-1))
        plot(Z_RE(:,1),Z_RE(:,2),'.','color',[0.2,0.2,0.2]);hold on
        subplot(2,3,3+3*(directoryIndex-1))
        plot(YAW_AE(:,1),YAW_AE(:,2),'.','color',[0.2,0.2,0.2]);hold on
    
    %% Plot rrmse and rmse
    subplot(2,3,1+3*(directoryIndex-1))
        plot(xy_rrmse,'color','k','linewidth',3);
    subplot(2,3,2+3*(directoryIndex-1))
        plot(z_rrmse,'color','k','linewidth',3);
    subplot(2,3,3+3*(directoryIndex-1))
        plot(yaw_rmse,'color','k','linewidth',3);
    %% Plot slope
    subplot(2,3,1+3*(directoryIndex-1))
        pxySlope = pxy(2)+pxy(1).*index;
        plot(index,pxySlope,'color','r','linewidth',1);
    subplot(2,3,2+3*(directoryIndex-1))
        pzSlope = pz(2)+pz(1).*index;
        plot(index,pzSlope,'color','r','linewidth',1);
    subplot(2,3,3+3*(directoryIndex-1))
        pyawSlope = pyaw(2)+pyaw(1).*index;
        plot(index,pyawSlope,'color','r','linewidth',1);
    
    
end

% Edit some axis and text properties
algorithmTitles={'Visual Odometry','Polynomial Regression'};
    subplot(2,3,1)
    title('xy')
    subplot(2,3,2)
    title({[algorithmTitles{nmbr3}, ' - Measured dataset'],'z'},'FontSize',15)
    subplot(2,3,3)
    title('yaw')
    subplot(2,3,4)
    title('x,y')
    subplot(2,3,5)
    title({[algorithmTitles{nmbr3}, ' - Simulated dataset'],'z'},'FontSize',15)
    subplot(2,3,6)
    title('yaw')



if nmbr3 == 1
    %XY
    xyaxis = [0,40,0,20];
    zaxis = [0,40,0,20];
    yawaxis = [0,40,0,0.4];
elseif nmbr3 == 2
    xyaxis = [0,40,0,20];
    zaxis = [0,40,0,20];
    yawaxis = [0,40,0,0.4];
end

    subplot(2,3,1)
        axis(xyaxis)
        ylabel('RRMSE [%(z)]','FontSize',14)
        xlabel('Sequence index','FontSize',14)
        legend('Relative error','RRMSE')
    subplot(2,3,4)
        axis(xyaxis)
        ylabel('RRMSE [%(z)]','FontSize',14)
        xlabel('Sequence index','FontSize',14)
        legend('Relative error','RRMSE')
    subplot(2,3,2)
        axis(zaxis)
        ylabel('RRMSE [%(z)]','FontSize',14)
        xlabel('Sequence index','FontSize',14)
        legend('Relative error','RRMSE')
    subplot(2,3,5)
        axis(zaxis)
        ylabel('RRMSE [%(z)]','FontSize',14)
        xlabel('Sequence index','FontSize',14)
        legend('Relative error','RRMSE')
    subplot(2,3,3)
        axis(yawaxis)
        ylabel('RMSE [rad]','FontSize',14)
        xlabel('Sequence index','FontSize',14)
        legend('Absolute error','RMSE')
    subplot(2,3,6)
        axis(yawaxis)
        ylabel('RMSE [rad]','FontSize',14)
        xlabel('Sequence index','FontSize',14)
        legend('Absolute error','RMSE')
      
outpath = '/Users/Fredrik/Google Drive/Kurser/Thesis/Documentation/Report_presentations/AndersonThesis/Texter/Results/Figures/';
%outpath = '../../../../../Documentation/Report_presentations/';
filetitle = ['Comparison3-indexederrors-',plotIdentifications{nmbr3}];
fulltitle = [outpath,filetitle,'.pdf'];        
        
   
%saveas(nmbr3,fulltitle)
print(nmbr3,'-dpdf','-bestfit',fulltitle)

disp(['ALGORITHM: ', plotIdentifications{nmbr3}])
disp(resultMatrix)
disp('Done')



