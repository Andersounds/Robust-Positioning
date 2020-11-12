
% CHOOSE  numbers according to the cell arrayw below
%directories={'20-04-09/','20-11-3-sim/'}; --nmbr1 = [1,2]
%datasets={'20-04-09-18/','20-04-09-23/','20-04-09-27/','20-04-09-28/'};%nmbr2=[1,2,3,4];
%algorithms={'VO','MARTON'};            % nmbr3=2; %Only choose one
%settings={'hl','hm','hh','mh','lh'};    %nmbr4=[3];%1,2,3 or 3,4,5 
%occlusions={'AZ60FB15','AZ10FB20','AZ5FB40'};% nmbr5=[1,2,3];% 1 or 2 or 3?

nmbr1=1;
nmbr2 = [1,2,3,4];
nmbr3=2;
nmbr4=[3];
nmbr5=[1,2,3];
basePath = '../data/';

[X,Y,Z,YAW,EXECT,GTX,GTY,GTZ,GTYAW,GTT] = extractFBSequences(nmbr1,nmbr2,nmbr3,nmbr4,nmbr5,basePath);





%At this point we have the complete matrices

%Get columns on by one
%Check isempty before calculating
index = 4;
figure

for i=1:runs %outer loop: every run (every file)
    for j=1:length(T{i}) %inner loop: every fallback sequence
    if ~isempty(X{i,index})
        err=X{i,index}-GTX{i,index};

        %err = ((X{i,index}-GTX{i,index}).^2 + (Y{i,index}-GTY{i,index}).^2).^(1/2);%RMSE
       %err  = EXECT{i,index};
        plot(err,'color',[0.95,0.95,0.95]);hold on;
        plot(err,'.','color','k');
        hold on;
    end
    end
end
title(algorithms{nmbr3})
xlabel('Fallback sequence index')
ylabel('Error [m]')
axis([0,40,-0.25,0.25])




