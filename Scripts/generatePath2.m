clear

% TODO: Heading, roll, pitch

fps = 20;   %Framerate (constant over whole simulation)
mps = 0.8;      %Speed [m/s]
wpass = 0.1;
pad = 30;   %Pre and post constant padding of signal to allow for filter risetime
xlim = 4;
ylim = 2;
zlim = -2;

line = [0.5,0.5,-1.7];% Start position x,y,z
%Create nx4 matrix of corner points. Loop through rows and add them [x,y,z,mps]
%          X    Y       Z       mps
corners = [2,   0.3,    -1.9,    0.8;
           2.2, 0.5,    -1.8,    0.8;
           2.5, 0.9,    -1.7,    0.6;
           2.7, 1.3,    -1.6,    0.8;
           2.5, 1.6,    -1.7,    0.8;
           2.0, 1.7,    -1.8,    0.6;
           1.5, 0.7,    -1.7,    0.9;
           1.3, 0.5,    -1.7,    0.2;
           1.1, 0.6,    -1.8,    0.8];
 

 %Pad to allow filter risetime
 for i=1:pad
    line = [line; line(1,:)]; 
 end
 %Add lines between all corners
 for i=1:length(corners(:,1))  
    line = addLine(line,corners(i,1:3),fps,corners(i,4));
 end
 %Pad to allow filter risetime
 for i=1:pad
    line = [line; line(1,:)]; 
 end
%Apply lowpass filter
LPline = lowpass(line,wpass,fps);

%Remove padding
line = line(pad:end-pad,:);
LPline = LPline(pad:end-pad,:);

%Calculate timestamps
frames = length(LPline(:,1));
t =linspace(0,frames/fps,frames)';
% Calculate ideal heading
rad = getHeading(LPline);
 
% Add roll/pitch
% Blimp usually swings at 0.85Hz with about +-0.03 rad pitch and +-0.01 rad
% roll
freq = 0.85;%Hz
roll = 0.03.*cos(t.*2*pi*freq);
pitch = 0.012.*sin(t.*2*pi*1.2*freq);


%Calculate dist
dist = LPline(:,3)./(cos(roll).*cos(pitch));


%Create figure
f = figure(1);
plot3(line(1,1),line(1,2),line(1,3),'o')
hold on
plot3(line(:,1),line(:,2),line(:,3))
hold on
axis([0,xlim,0,ylim,zlim,0]);


%b = 1;
%a = [1, 0.1];
%LPline = filtfilt(b,a,line(:,2:4));
%LPline = filter(b,a,line(:,2:4));

plot3(LPline(:,1),LPline(:,2),LPline(:,3),'b')
legend('Start','Corners','Lowpass');
xlabel('X [m]');ylabel('Y [m]');
%f.CurrentAxes.ZDir = 'Reverse'


figure
subplot(3,1,1)
    plot(line(:,1))
    hold on
    plot(LPline(:,1))
subplot(3,1,2)
    plot(line(:,2))
    hold on
    plot(LPline(:,2))
subplot(3,1,3)
    plot(line(:,3))
    hold on
    plot(LPline(:,3))
    hold on
    plot(dist)
    legend('line','path','dist')
    
figure
plot(rad)
figure
plot(t,roll)

total = [t';LPline';roll';pitch';rad';dist']';% Write to csv file

csvwrite('path-200331-1.csv',total)