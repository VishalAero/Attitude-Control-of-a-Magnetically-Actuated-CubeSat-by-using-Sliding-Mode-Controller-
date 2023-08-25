function diff_states  = Model1(time,state)
CubeSat_param1;

%% Extracting the values from variable States
q = state(1:4);
Omega = state(5:7);

%% Controller
% Desired values
q_d = [1;0;0;0] ;
Omega_d = [0;0;0] ;

%% Controller Torq
% Magnetic field vector
b_x = 0.0000000001;      %Magnetic field magnitude in spacecraft x-axis
b_y = 0.0000000001;      %Magnetic field magnitude in spacecraft y-axis
b_z = 0.0000000001;      %Magnetic field magnitude in spacecraft z-axis
B = [b_x;b_y;b_z];

%% Gravity Gradient torq
Omega_orbit = 0.0012;  % Orbital velocity of spacecraft z-axis

Mu_E = (3.98600442)*10^(14) ;% [m3 s−2]; %Specific gravitational constant for Earth (4𝜋2 AU3/year2)
RxE_SC = (1*10^4)*cos(Omega_orbit*time); % Distance from spacecraft center of gravity to Earths center of gravity in spacecraft x-axis
RyE_SC = (1*10^4)*sin(Omega_orbit*time); % Distance from spacecraft center of gravity to Earths center of gravity in spacecraft y-axis
RzE_SC = 0;                              % Distance from spacecraft center of gravity to Earths center of gravity in spacecraft z-axis
RE_SC = [RxE_SC ; RyE_SC ; RzE_SC];      % Distance from spacecraft center of gravity to Earths center of gravity

x = (RE_SC.^5);
Torq_G = x\((3*Mu_E).*([(RyE_SC*RzE_SC)*(I_z-I_y)  ;(RxE_SC*RzE_SC)*(I_x-I_z)  ;(RxE_SC*RyE_SC)*(I_y-I_x)]));

%% CubeSat Mathematical Model
Omega_0 = [0;  0;  Omega_orbit]; % Orbital Reference Frame

% Quaternion Rotation Matrix from Orbital to Body
A_q = [ (q(1)^2)+(q(2)^2)-(q(3)^2)-(q(4)^2) , 2*(q(2)*q(3) + q(2)*q(4))           , 2*(q(2)*q(4) - q(1)*q(3)) 
        2*(q(2)*q(3) - q(1)*q(4))           , (q(1)^2)-(q(2)^2)+(q(3)^2)-(q(4)^2) , 2*(q(3)*q(4) + q(1)*q(2))  
        2*(q(2)*q(4) + q(1)*q(3))           , 2*(q(3)*q(4) - q(1)*q(2))           , (q(1)^2)-(q(2)^2)-(q(3)^2)+(q(4)^2)];
Omega_r = Omega - (A_q *Omega_0); % Body Reference Frame

quat = quaternion([q(1),q(2),q(3),q(4)]);
wquat = quaternion([0,Omega_r(1),Omega_r(2),Omega_r(3)]);
q_diff = 0.5*quat*wquat; % Calculating in Body Frame
[a,b,c,d] = parts(q_diff);
q_diff = [a,b,c,d];

%% SMC
ss = Omega_r + q(2:4)./q(1);
d_nn = q_diff(1);
d_ee = (q_diff(2:4))';
Torq_SMC = - II*(((d_ee*q(1)) - (q(2:4)*d_nn))./(q(1)^2) + (K1*sign(ss) + K2*ss + K3)) + cross(Omega_r,(II*Omega_r));
Torq_desire = Torq_SMC;

% Magnetic moment vector
M = cross(B,Torq_desire)/(norm(B)^2);
Torq_C = cross(M,B);
Torq = Torq_C + Torq_G ;

%%
Omega_diff = II\(Torq - cross(Omega_r,(II*Omega_r))); % Calculating in Body Frame
diff_states = [q_diff'; Omega_diff];

end