# Battle Drones

National University of Singapore AY2023-24
EE2028 Microcontroller Programming and Interfacing Assignment 2

Board used: STM32 B-L475VE-IOT01A

What does it do? (Please read comments in code for more details, below is only a summary of the features of this drone)

STANDBY_MODE:
1. LED always on
2. Send telemetry readings and monitoring every 1s
3. Single press: ignored
4. Double press: BATTLE_MODE

BATTLE_MODE without Last of EE2028:
1. LED toggle every 1s
2. Send telemetry readings and monitoring every 1s
3. Single press: charge lasergun and fire when there is sufficient energy
4. Double press: STANDBY_MODE
 
BATTLE_MODE with Last of EE2028 (when drone is flipped)
1. Timeout: 10s
2. Single press: ignored
3. Double press: rescue drone; BATTLE_MODE without Last of EE2028
4. If not rescued in 10s, program will be terminated. Last message will be sent and Command Center can respond last message

Other features:
1. WiFi-based communication (SPI interface)
2. UART transmit
3. Modular and highly efficient code for easy debugging and adding of new features

To future students of EE2028, please feel free to read the code even if the assignment is different to gain inspiration on coding structures and logic. By no means should you copy my code wholesale and assume it will work, the code is still answering the questions and achieving the objectives of my assignment. Do take some time to explore the board and the libraries, experience some headaches and have fun:P
