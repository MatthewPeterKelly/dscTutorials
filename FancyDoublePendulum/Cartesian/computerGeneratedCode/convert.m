function DataOut = convert(DataIn)
% function DataOut = convert(DataIn)
%
% Computer Generated File -- DO NOT EDIT
%
% This function was created by the function Write_Convert()
% 13-Dec-2013 11:41:00
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
            nData = size(DataIn.x0,1);    
            DataOut = zeros(nData,12);
            DataOut(:,1) = DataIn.x0; % (m) Hip horizontal position
            DataOut(:,2) = DataIn.y0; % (m) Hip vertical position
            DataOut(:,3) = DataIn.x1; % (m) Foot One horizontal position
            DataOut(:,4) = DataIn.y1; % (m) Foot One vertical position
            DataOut(:,5) = DataIn.x2; % (m) Foot Two horizontal position
            DataOut(:,6) = DataIn.y2; % (m) Foot Two vertical position
            DataOut(:,7) = DataIn.dx0; % (m/s) Hip horizontal velocity
            DataOut(:,8) = DataIn.dy0; % (m/s) Hip vertical velocity
            DataOut(:,9) = DataIn.dx1; % (m/s) Foot One horizontal velocity
            DataOut(:,10) = DataIn.dy1; % (m/s) Foot One vertical velocity
            DataOut(:,11) = DataIn.dx2; % (m/s) Foot Two horizontal velocity
            DataOut(:,12) = DataIn.dy2; % (m/s) Foot Two vertical velocity
        case 5 % Actuators
            nData = size(DataIn.F1,1);    
            DataOut = zeros(nData,5);
            DataOut(:,1) = DataIn.F1; % (N) Compresive axial force in Leg One
            DataOut(:,2) = DataIn.F2; % (N) Compresive axial force in Leg Two
            DataOut(:,3) = DataIn.T1; % (Nm) External torque applied to Leg One
            DataOut(:,4) = DataIn.T2; % (Nm) External torque applied to Leg Two
            DataOut(:,5) = DataIn.Thip; % (Nm) Torque acting on Leg Two from Leg One
        case 4 % Contacts
            nData = size(DataIn.H1,1);    
            DataOut = zeros(nData,4);
            DataOut(:,1) = DataIn.H1; % (N) Foot One, horizontal contact force
            DataOut(:,2) = DataIn.V1; % (N) Foot One, vertical contact force
            DataOut(:,3) = DataIn.H2; % (N) Foot Two, horizontal contact force
            DataOut(:,4) = DataIn.V2; % (N) Foot Two, vertical contact force
    otherwise 
        error('Invalid number of channels in input.')
    end 
    
else    
    %Then converting from a matrix to a struct    
    switch size(DataIn,2)    
        case 12 % States
            DataOut.x0 = DataIn(:,1); % (m) Hip horizontal position
            DataOut.y0 = DataIn(:,2); % (m) Hip vertical position
            DataOut.x1 = DataIn(:,3); % (m) Foot One horizontal position
            DataOut.y1 = DataIn(:,4); % (m) Foot One vertical position
            DataOut.x2 = DataIn(:,5); % (m) Foot Two horizontal position
            DataOut.y2 = DataIn(:,6); % (m) Foot Two vertical position
            DataOut.dx0 = DataIn(:,7); % (m/s) Hip horizontal velocity
            DataOut.dy0 = DataIn(:,8); % (m/s) Hip vertical velocity
            DataOut.dx1 = DataIn(:,9); % (m/s) Foot One horizontal velocity
            DataOut.dy1 = DataIn(:,10); % (m/s) Foot One vertical velocity
            DataOut.dx2 = DataIn(:,11); % (m/s) Foot Two horizontal velocity
            DataOut.dy2 = DataIn(:,12); % (m/s) Foot Two vertical velocity
        case 5 % Actuators
            DataOut.F1 = DataIn(:,1); % (N) Compresive axial force in Leg One
            DataOut.F2 = DataIn(:,2); % (N) Compresive axial force in Leg Two
            DataOut.T1 = DataIn(:,3); % (Nm) External torque applied to Leg One
            DataOut.T2 = DataIn(:,4); % (Nm) External torque applied to Leg Two
            DataOut.Thip = DataIn(:,5); % (Nm) Torque acting on Leg Two from Leg One
        case 4 % Contacts
            DataOut.H1 = DataIn(:,1); % (N) Foot One, horizontal contact force
            DataOut.V1 = DataIn(:,2); % (N) Foot One, vertical contact force
            DataOut.H2 = DataIn(:,3); % (N) Foot Two, horizontal contact force
            DataOut.V2 = DataIn(:,4); % (N) Foot Two, vertical contact force
    otherwise 
        error('Invalid number of channels in input.')
    end 
    
end    
    
end    
