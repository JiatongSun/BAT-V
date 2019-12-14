# MEAM 510 Final Project Code



__*Team Member: Jiatong Sun, XInyue Wei, Haojiong Lu*__



Our code for the 2019 final project is put in the __FinalBot__ directory. The basic structure of the __FinalBot__ is shown below.

| Controller     | MainBoard     | AuxiliaryBoard       | Unused         |
| -------------- | ------------- | -------------------- | -------------- |
| Controller.ino | MainBoard.ino | AuxiliaryBoard.ino   | encoder        |
|                | library files | audio_example_file.h | pid_control    |
|                |               |                      | stepper_motor  |
|                |               |                      | uart           |
|                |               |                      | ultrosonic_ISR |
|                |               |                      | vive_ISR       |

1. In the __Controller__ directory, the `Controller.ino` includes code for the remote controller. 

2. In the __MainBorad__ directory, there exist a `MainBoard.ino` and a great number of library files, all of which are from the __DemoBot__. No extra library has been used for this project.

   In the `MainBoard.ino`, all functions created by ourselves have been wrapped between three long horizontal lines indicating the beginning of a function and another three lines indicating the end of a function. An example is shown below.

   ```
   //=====================================================================================
   //=================================== library start ===================================
   //=====================================================================================
   #include <WiFi.h>
   #include <WiFiUDP.h>
   //=====================================================================================
   //==================================== library end ====================================
   //=====================================================================================
   ```

   Note that the line wrapping our own functions is much longer than the ones originally commented in the ``DemoBot.ino`` so it's easy to distinguish our functions from others.

3. In the __AuxiliaryBoard__ directory, there exists an ``audio_example_file.h``, which is the 16-bit file for a laughter sound.

4. In the __Unused__ directory, there are six subdirectories in total, which contain the functionalities we tested but didn't use eventually. Additionally, the __uart__ directory contains two subdirectories, of which one is for ESP32 and the other is for Teensy.

5. All code in the __FinalBot__ has been well commented for better reading.

6. More details of the code can be found in our final report section 4: __Processor Architecture & Code Architecture__.







