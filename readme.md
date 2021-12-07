# Arduino RC project
*Media data link: [GoogleDrive]()*  
   
It's small and lightweight project to implement RC of handmade plane  
  
Hardware:   
    - arduino Nano (transmitter), arduino Micro (receiver);  
    - NRF24L01 2.4 MHz radio module;  
    - PWM dual H-Bridge L298N drievr - 2pcs.  
    - DC-DC 3V to 5V 1A converter (powering servo)  
    - N30 Motor 340000 RPM - 2pcs.  
    - li-ion 700 mAh battery for controller (transmiter), li-po 450 mAh battery for plane (receiver)  
    - styrofoam hand throw airplane 
      
Software:  
    - arduino IDE;  
    - AlexGyver libraries.  
  
Pending optimisation. TODO:  
    - refuse of using GyverServo library and switch to hardware timers to control servo to speed up response;  
    - redo battery charge control function;  
    - implement flaps control;  
    - implement trimmers.  


