I_x = 8.817*(10^-4);%principal moment of inertia corresponding to spacecraft x-axis
I_y = 9.804*(10^-4);%principal moment of inertia corresponding to spacecraft y-axis
I_z = 1.035*(10^-3);%principal moment of inertia corresponding to spacecraft z-axis
II = [I_x  0     0
       0  I_y   0
       0    0   I_z];   %Principal moment of inertia vector

n = 1 ; % Number of coil rotation
Areax = 1; % Area of x plane of CubeSat
Areay = 1; % Area of y plane of CubeSat
Areaz = 1; % Area of z plane of CubeSat