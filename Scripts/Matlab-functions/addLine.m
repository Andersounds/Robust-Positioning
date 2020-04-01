%Give path containing at least one row-coordinate and timestamp as:
% path = [---;
%         ---;
%         t,x,y,z];
% The function draws a line from last row timestamp and coordinate to coord
% with proper timestamps, fps and velocity

% Maybe t should be given later after filtering is done?
function data2 = addLine(data,to,fps,mps)
siz = size(data);
if siz(2)~= 3 
    data2=[];
    disp('GIVEN DATA IS OF WRONG FORMAT. ([x,y,z])');
    return
end
from = data(end,:);
%t1 = data(end,1);
dist = norm(to-from);   %Physical distance
dt = dist/mps;          %Time between points
frames = fps*dt;        %Number of frames

xline = linspace(from(1),to(1),frames+1);
yline = linspace(from(2),to(2),frames+1);
zline = linspace(from(3),to(3),frames+1);
%t = linspace(t1,t1+dt,frames+1);%Timestamp vector

line = [xline;yline;zline]';% New data. columns
data2 = [data;line(2:end,:)];%Skip forst row as it is same as last of prev
disp('Check overgang. ')

end