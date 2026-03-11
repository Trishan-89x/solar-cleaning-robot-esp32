# Solar Bot Web Control (New Bot)

The new robot control interface is deployed online.

Web App:
https://solarbotwebapp.vercel.app

This interface communicates with the ESP32 robot through HTTP commands over WiFi.

The ESP32 runs a local web server and receives commands such as:

fXXX  → move forward  
rXXX  → move reverse  
b     → brake  

where XXX is the PWM speed value (0–255).
