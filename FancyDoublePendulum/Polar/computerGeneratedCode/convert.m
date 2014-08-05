function DataOut = convert(DataIn)
% function DataOut = convert(DataIn)
%
% Computer Generated File -- DO NOT EDIT
%
% This function was created by the function Write_Convert()
% 10-Dec-2013 19:40:47
%
% This function will convert between special structs and matricies. 
% The input must match exactly with one of the cases below.
%
% Matthew Kelly
% Cornell University
%
%
if isstruct(DataIn)    
    %Then converting from a struct to a matrix    
    Names = fieldnames(DataIn);    
    switch length(Names)    
        case 12 % States
            nData = size(DataIn.x,2);    
            DataOut = zeros(12,nData);
            DataOut(1,:) = DataIn.x; % (m) Foot One horizontal position
            DataOut(2,:) = DataIn.y; % (m) Foot One vertical position
            DataOut(3,:) = DataIn.th1; % (rad) Leg One absolute angle
            DataOut(4,:) = DataIn.th2; % (rad) Leg Two absolute angle
            DataOut(5,:) = DataIn.L1; % (m) Leg One length
            DataOut(6,:) = DataIn.L2; % (m) Leg Two length
            DataOut(7,:) = DataIn.dx; % (m/s) Foot One horizontal velocity
            DataOut(8,:) = DataIn.dy; % (m/s) Foot One vertical velocity
            DataOut(9,:) = DataIn.dth1; % (rad/s) Leg One absolute angular rate
            DataOut(10,:) = DataIn.dth2; % (rad/s) Leg Two absolute angular rate
            DataOut(11,:) = DataIn.dL1; % (m/s) Leg One extension rate
            DataOut(12,:) = DataIn.dL2; % (m/s) Leg Two extensioin rate
        case 5 % Actuators
            nData = size(DataIn.F1,2);    
            DataOut = zeros(5,nData);
            DataOut(1,:) = DataIn.F1; % (N) Compresive axial force in Leg One
            DataOut(2,:) = DataIn.F2; % (N) Compresive axial force in Leg Two
            DataOut(3,:) = DataIn.T1; % (Nm) External torque applied to Leg One
            DataOut(4,:) = DataIn.T2; % (Nm) External torque applied to Leg Two
            DataOut(5,:) = DataIn.Thip; % (Nm) Torque acting on Leg Two from Leg One
        case 4 % Contacts
            nData = size(DataIn.H1,2);    
            DataOut = zeros(4,nData);
            DataOut(1,:) = DataIn.H1; % (N) Foot One, horizontal contact force
            DataOut(2,:) = DataIn.V1; % (N) Foot One, vertical contact force
            DataOut(3,:) = DataIn.H2; % (N) Foot Two, horizontal contact force
            DataOut(4,:) = DataIn.V2; % (N) Foot Two, vertical contact force
    otherwise 
        error('Invalid number of channels in input.')
    end 
    
else    
    %Then converting from a matrix to a struct    
    switch size(DataIn,1)    
        case 12 % States
            DataOut.x = DataIn(1,:); % (m) Foot One horizontal position
            DataOut.y = DataIn(2,:); % (m) Foot One vertical position
            DataOut.th1 = DataIn(3,:); % (rad) Leg One absolute angle
            DataOut.th2 = DataIn(4,:); % (rad) Leg Two absolute angle
            DataOut.L1 = DataIn(5,:); % (m) Leg One length
            DataOut.L2 = DataIn(6,:); % (m) Leg Two length
            DataOut.dx = DataIn(7,:); % (m/s) Foot One horizontal velocity
            DataOut.dy = DataIn(8,:); % (m/s) Foot One vertical velocity
            DataOut.dth1 = DataIn(9,:); % (rad/s) Leg One absolute angular rate
            DataOut.dth2 = DataIn(10,:); % (rad/s) Leg Two absolute angular rate
            DataOut.dL1 = DataIn(11,:); % (m/s) Leg One extension rate
            DataOut.dL2 = DataIn(12,:); % (m/s) Leg Two extensioin rate
        case 5 % Actuators
            DataOut.F1 = DataIn(1,:); % (N) Compresive axial force in Leg One
            DataOut.F2 = DataIn(2,:); % (N) Compresive axial force in Leg Two
            DataOut.T1 = DataIn(3,:); % (Nm) External torque applied to Leg One
            DataOut.T2 = DataIn(4,:); % (Nm) External torque applied to Leg Two
            DataOut.Thip = DataIn(5,:); % (Nm) Torque acting on Leg Two from Leg One
        case 4 % Contacts
            DataOut.H1 = DataIn(1,:); % (N) Foot One, horizontal contact force
            DataOut.V1 = DataIn(2,:); % (N) Foot One, vertical contact force
            DataOut.H2 = DataIn(3,:); % (N) Foot Two, horizontal contact force
            DataOut.V2 = DataIn(4,:); % (N) Foot Two, vertical contact force
    otherwise 
        error('Invalid number of channels in input.')
    end 
    
end    
    
end    
