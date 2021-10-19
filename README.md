# ILSF Documentation

The `src` directory contains different applications that were presented at the ILSF (I Love Science Festival).

* In `src\atmel_studio_project` an example of Arduino project ported to Atmel Studio can be found. Using Atmel Studio as an IDE offers you a higher flexibility and transparence over your code. The script `avrdude_program_nano_every.bat` is an example on how to program an Arduino Nano Every board directly from the IDE. More informations on how to setup an external programmer in Atmel Studio can be found [here](http://web.engr.oregonstate.edu/~jinyo/ece375/pdf/Setup_External_programmer_in_Atmel_Studio.pdf).


* In `src\max3010_main_code_arduino` there is the Arduino project used to read data from the MAX301x PPG sensor, compute the HR and SpO2 and send it through serial interface. It is the main code for the `PPG Dashboard` application and for `3D PPG Printing` application.


* In `src\ping_pong_tower_code_arduino` there is the Arduino project used to control the ping pong tower based on the heart rate value.

* In `src\python_3d_printing` can be found the Python application used to acquire and 3D print the PPG signal. 

* In `src\python_dashboard` and `src\python_dashboard_2` can be found the Python dashboard application.

Run the ping pong tower application
-----------------------------------
* Firstly, one has to read the documentation of the ping pong tower to get a clear picture how it works, how the height measurement is done, what are the sensor output options and how to calibrate the height sensor. The documentation can be found in `doc` directory.
* Go to the Arduino project and analyse the code, try to configure the configuration macros according to your needs. Also do not forget to set the `ANALOG_PIN` and `PWM_CONTROL_PIN` according to the design.


![image](https://user-images.githubusercontent.com/24388880/137904320-08df7fbf-30b2-4bb7-bfd6-54fbe8f9ee59.png)

* Compile and load the code from `src\ping_pong_tower_code_arduino\ping_pong_main_code_arduino` to the Arduino. 

* Perform the calibration as described in `doc\height sensor calibration`. After calibration (do not power off the ping pong tower), disconnect the fan and connect it to the extension board.

* Connect the PWM pin and the sensor output to the Arduino. The ball should start rising and float in the middel of the tube.


* If the ball is not stable or the height is does not correspond you might need to create by yourself another the following look-up tables. In this case a voltage divider, so the voltage values should be changed if the ping pong tower output is connected directly to the `ANALOG_PIN`.
 
![image](https://user-images.githubusercontent.com/24388880/137905638-c2d91dd0-4c33-493b-bcca-d7aafe7d3fc6.png)

* To create the voltage lookup table one can use the `src\ping_pong_tower_code_arduino\set_pwm_manually` project and which allows to set manually the ball at a specific height (the height can also be changed covering the fan with your hand).  Read the sensor analog values and create the `hr_lookup_table` and `voltage_lookup_table` according to your needs.

* To test that the newly created lookup tables, set the `SET_MANUAL_BEAT_AVG` to `TRUE`, compile, load and from a serial terminal send the desired HR value. The ball should be floating at the desired height according to the serial input value.


Run the 3D printing application
--------------------------------

* Install the requirements list from `src\python_3d_printing` 

```bash
cd src\python_3d_printing
pip install -r requirements.txt
```

