function [X,P] = Initial_State(P)

%This function generates a guess at the state struct for fmincon

%Rename for easy reading:
  angleGuess = P.init.angleGuess;
  rateGuess = P.init.rateGuess;
  torqueGuess = P.init.torqueGuess;
  durationGuess = P.init.durationGuess;
  timeGuess = linspace(0,durationGuess,length(angleGuess));

  
%Now interpolate between the guess points:
  nGridPts = P.nGridPts;
  Time = linspace(0, durationGuess, nGridPts);
  
  angle = interp1(timeGuess',angleGuess',Time')';
  rate = interp1(timeGuess',rateGuess',Time')';
  torque = interp1(timeGuess',torqueGuess',Time')';
  
%Store the guess as a struct:  
  X.duration = durationGuess;
  X.state = [angle;rate];
  X.torque = torque;
 
end