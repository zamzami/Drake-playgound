
%%% Definitions
% Generalized coordinates...
syms x y phi alpha l
q    = [x y phi alpha l];
% ... and generalized speeds
syms dx dy dphi dalpha dl  
dqdt = [dx dy dphi dalpha dl];

% Define the necessary parameter subset:
% Gravity
syms g
% Segment dimensions:
syms l2 l3 rFoot
% Masses/Inertia;
syms m1 m2 m3
syms j1 j2 j3
param = [l2 l3 rFoot g m1 m2 m3 j1 j2 j3];

  % Parameter of the model
%     systParam.l_0       = 1;     % [l_0] uncompressed leg length
%     systParam.alpha_0   = 0;     % [rad] resting leg angle
%     systParam.m1        = 0.85;  % [m_0] mass of the main body
%     systParam.m2        = 0.10;  % [m_0] mass of the upper leg segment
%     systParam.m3        = 0.05;  % [m_0] mass of the lower leg segment
%     systParam.l2        = 0.25;  % [l_0] distance between hip joint and CoG of the upper leg segment
%     systParam.l3        = 0.25;  % [l_0] distance between foot point and CoG of the lower leg segment
%     systParam.rFoot     = 0.05;  % [l_0] foot radius
%     systParam.j1        = 0.4;   % [m_0*l_0^2] inertia of the main body
%     systParam.j2        = 0.002; % [m_0*l_0^2] inertia of the upper leg segment
%     systParam.j3        = 0.002; % [m_0*l_0^2] inertia of the lower leg segment
%     systParam.kalpha    = 5;     % [m_0*g*l_0/rad] rotational spring stiffness in the hip joint  
%     systParam.balphaRat = 0.2;   % [*] damping ratio.  Use 20% of critical damping
%     systParam.kl        = 15;    % [m_0*g/l_0] linear spring stiffness in the prismatic joint 
%     systParam.blRat     = 0.2;   % [*] damping ratio.  Use 20% of critical damping
   

t781 = alpha+phi;
t782 = cos(t781);
t783 = l.*m3;
t784 = l2.*m2;
t789 = l3.*m3;
t785 = t783+t784-t789;
t786 = t782.*t785;
t787 = m1+m2+m3;
t788 = sin(t781);
t790 = t785.*t788;
t791 = l.^2;
t792 = m3.*t791;
t793 = l2.^2;
t794 = m2.*t793;
t795 = l3.^2;
t796 = m3.*t795;
t798 = l.*l3.*m3.*2.0;
t797 = j2+j3+t792+t794+t796-t798;
t799 = m3.*t788;
M = reshape([t787,0.0,t786,t786,t799,0.0,t787,t790,t790,-m3.*t782,t786,t790,j1+j2+j3+t792+t794+t796-l.*l3.*m3.*2.0,t797,0.0,t786,t790,t797,t797,0.0,t799,-m3.*t782,0.0,0.0,m3],[5, 5]);
