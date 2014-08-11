function Z = Sensors(State,Params)

%Directly measure three of the states, estimate overall orientation
H = Params.Est.H;

%Apply random noise
w = Params.Est.Sensor_Noise_Variance.*randn(size(H,1),1);

Z = H*State + w; 

end