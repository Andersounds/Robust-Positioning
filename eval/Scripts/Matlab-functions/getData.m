%This function is gets the data contained in the passed file
%path. Data columns are hardcoded

function [t x y z roll pitch yaw modes] = getData(filepath)
%Data columns
tcol = 1;
xcol = 2;
ycol = 3;
zcol = 4;
rollcol = 5;
pitchcol = 6;
yawcol = 7;
modeCol = 9;

data= csvread(filepath,1,0);
% Create data vectors 
t = data(:,tcol)./1000;
x = data(:,xcol);
y = data(:,ycol);
z = data(:,zcol);
roll = data(:,rollcol);
pitch = data(:,pitchcol);
yaw = data(:,yawcol);
% Extract modes
modes = data(:,modeCol);
end