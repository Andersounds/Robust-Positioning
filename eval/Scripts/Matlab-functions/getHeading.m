%calculate normed gradient at each (row) point in given data. 
%Last point is copy of second last point to keep length compatibility
%Calculate plane heading from vector
function rad = getHeading(data)

rad = [];
for i=1:(length(data(:,1))-1)
    dir_ =data(i+1,1:2)-data(i,1:2);%only consider x-y i.e rot around z
    dir = dir_./norm(dir_);%Normalized length
    angle = acos(dir(1));% Get angle. But answer can also be 2pi-angle
    if(dir(2)<0)
       angle = 2*pi - angle; %Bottom halv of unit circle 
    end
    rad = [rad;angle];

end
%Add last row
rad = [rad;rad(end)];

end