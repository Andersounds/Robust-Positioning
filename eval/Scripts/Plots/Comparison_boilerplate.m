
% CHOOSE  numbers according to the cell arrayw below
%directories={'20-04-09/','20-11-3-sim/'}; --nmbr1 = [1,2]
%datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};%nmbr2=[1,2,3,4];
%algorithms={'VO','MARTON'};            % nmbr3=2; %Only choose one
%settings={'hl','hm','hh','mm','mh','lm','lh'};    %nmbr4=[3];%
%occlusions={'AZ30FB15','AZ10FB20','AZ5FB40'};% nmbr5=[1,2,3];% 1 or 2 or 3?

nmbr1=[1];          % Only collected dataset
nmbr2 = [1,2,3,4];  % All datasets
nmbr4=[2];          % 
nmbr5=[1,2,3];      %
runs = length(nmbr1)*length(nmbr2)*length(nmbr4)*length(nmbr5);
basePath = '../data/';


%% Visual odometry
nmbr3=1;
[X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,T] = extractFBSequences(nmbr1,nmbr2,nmbr3,nmbr4,nmbr5,basePath);

%At this point we have the complete matrices

%Get columns on by one
%Check isempty before calculating
figure
for i=1:runs %outer loop: every run (every file)
    for j=1:length(T(i,:)) %inner loop: every fallback sequence
    if isempty(T(i,j)) break; end %Need to check this because if one row is longer the others are filled with empty cells;   
   
    errXY = ((X{i,j}-GTX{i,j}).^2 + (Y{i,j}-GTY{i,j}).^2).^(1/2);%RMSE
    errZ=abs(Z{i,j}-GTZ{i,j});
    %Lite balett för att inte få +-2pi fel
    diffYAW = YAW{i,j}-GTYAW{i,j};
    toobigindexes = find(diffYAW>3);
    diffYAW(toobigindexes) = diffYAW(toobigindexes)-2*pi;
    toosmallindexes = find(diffYAW<-3);
    diffYAW(toosmallindexes) = diffYAW(toosmallindexes)+2*pi;
    errYAW = abs(diffYAW);
    
    subplot(2,3,1)
    %plot(errXY);hold on;
      plot(errXY,'color',[0.95,0.95,0.95]);hold on;
      plot(errXY,'.','color','k');hold on;
      axis([0,40,0,0.3]);
      xlabel('Fallback sequence index')
      ylabel('Error [m]')
      title('Visual Odometry - Horizontal RMSE')
    subplot(2,3,2)
    %plot(errZ);hold on;
      plot(errZ,'color',[0.95,0.95,0.95]);hold on;
      plot(errZ,'.','color','k');hold on;
      axis([0,40,0,0.3])
      xlabel('Fallback sequence index')
      ylabel('Error [m]')
      title('Visual Odometry - Vertical')
    subplot(2,3,3)
    %plot(errYAW);hold on;
      plot(errYAW,'color',[0.95,0.95,0.95]);hold on;
      plot(errYAW,'.','color','k');hold on;
      axis([0,40,0,0.3]);
      xlabel('Fallback sequence index')
      ylabel('Error [rad]')
      title('Visual Odometry - YAW')
    end
end




%% Polynomial regression - marton
nmbr3=2;
[X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,T] = extractFBSequences(nmbr1,nmbr2,nmbr3,nmbr4,nmbr5,basePath);

%At this point we have the complete matrices

%Get columns on by one
%Check isempty before calculating

for i=1:runs %outer loop: every run (every file)
    for j=1:length(T(i,:)) %inner loop: every fallback sequence
    if isempty(T(i,j)) break; end %Need to check this because if one row is longer the others are filled with empty cells;   

    errXY = ((X{i,j}-GTX{i,j}).^2 + (Y{i,j}-GTY{i,j}).^2).^(1/2);%RMSE
    errZ=abs(Z{i,j}-GTZ{i,j});
    
    diffYAW = YAW{i,j}-GTYAW{i,j};
    toobigindexes = find(diffYAW>3);
    diffYAW(toobigindexes) = diffYAW(toobigindexes)-2*pi;
    toosmallindexes = find(diffYAW<-3);
    diffYAW(toosmallindexes) = diffYAW(toosmallindexes)+2*pi;
    errYAW = abs(diffYAW);
    
    subplot(2,3,4)
    %plot(errXY);hold on;
      plot(errXY,'color',[0.95,0.95,0.95]);hold on;
      plot(errXY,'.','color','k');hold on;
      axis([0,40,0,0.3]);
      xlabel('Fallback sequence index')
      ylabel('Error [m]')
      title('Polynomial regression - Horizontal RMSE')
    subplot(2,3,5)
      %plot(errZ);hold on;
      plot(errZ,'color',[0.95,0.95,0.95]);hold on;
      plot(errZ,'.','color','k');hold on;
      axis([0,40,0,0.3])
      xlabel('Fallback sequence index')
      ylabel('Error [m]')
      title('Polynomial regression - Vertical')
    subplot(2,3,6)
      %plot(errYAW);hold on;
      plot(errYAW,'color',[0.95,0.95,0.95]);hold on;
      plot(errYAW,'.','color','k');hold on;
      axis([0,40,0,0.3]);
      xlabel('Fallback sequence index')
      ylabel('Error [rad]')
      title('Polynomial regression - YAW')
    end
end




