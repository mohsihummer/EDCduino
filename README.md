# EDCduino - pre-Beta
Arduino based open source Diesel Interceptor ECU- crank angle & rail pressure interception, 2xPWM channels  		
Designed for CRDI  & GDI engines  
Intercepts crank position and camposition sensors to change Start of Injection  
Alpha-N and Speed-Density  
Uses a digipot for rail pressure sensor interception  
Logs: Engine RPM, wideband O2, Accelerator pedal, MAP, thermisters  
Outputs 2x PWM channels (called VVT and boost)  
Based on Speeduino by Josh Stewart- https://github.com/noisymime/speeduino  
Uses TunerStudio as interface  
Configured for 60-2 crank, 1 teeth cam , cooper CRDI Turbo gensets designed by Ricardo  
Tested on Proteus, upto 3333rpm, at rate of change of 60 degs per rpm
