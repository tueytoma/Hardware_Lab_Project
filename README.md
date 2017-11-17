# Hardware_Lab_Project
Learning and testing various sensors of [STM32DISCOVERYF4](http://www.st.com/en/evaluation-tools/stm32f4discovery.html) board to bring together a project . This project to determine your position and the point of beginning. Relies on the direction that shifts the board.

**This project is to make use 3 sensor devices.**

**1. LIS302DL:** 3-axis accelerometer

**2. MP45DT02:** digital microphone

**3. CS43L22:** audio DAC, speaker driver

## LIS302DL: 3-axis accelerometer
The **LIS302DL** is an ultra compact low-power three axes linear accelerometer. It includes a sensing element and an IC interface able to provide the measured acceleration to the external world through I2C/SPI serial interface.

> [View Code](https://github.com/tueytoma/Hardware_Lab_Project/tree/master/Sensors%205730625221/LIS302DL) 

**Datasheet**

http://www.st.com/resource/en/datasheet/lis302dl.pdf

**Video**

https://www.youtube.com/watch?v=IbuJDEGVJcQ&feature=youtu.be

## MP45DT02: digital microphone
The **MP45DT02** is a compact, low-power, topport, omnidirectional, digital MEMS microphone. The MP45DT02 is built with a sensing element and an IC interface with stereo capability.

The sensing element, capable of detecting acoustic waves, is manufactured using a specialized silicon micromachining process to produce audio sensors. 

The IC interface is manufactured using a CMOS process that allows designing a dedicated circuit able to provide a digital signal externally in PDM format.

> [View Code](https://github.com/tueytoma/Hardware_Lab_Project/tree/master/Sensors%205730625221/MP45DT02)

**Datasheet**

http://www.st.com/resource/en/datasheet/mp45dt02-m.pdf

**Video**

https://www.youtube.com/watch?v=9F-v_QhTZV4&feature=youtu.be

## CS43L22: audio DAC, speaker driver
The **CS43L22** is a highly integrated, low power stereo DAC with headphone and Class D speaker amplifiers. The CS43L22 offers many features suitable for low power, portable system applications 

The DAC output path includes a digital signal processing engine with various fixed function controls. Tone Control provides bass and treble adjustment of four selectable corner frequencies. Digital Volume controls may be configured to change on soft ramp transitions while the analog controls can be configured to occur on every zero crossing. The DAC also includes de-emphasis, limiting functions and a BEEP generator delivering tones selectable across a range of two full octaves

> [View Code](https://github.com/tueytoma/Hardware_Lab_Project/tree/master/Sensors%205730625221/CS43L22)

**Datasheet**

https://www.cirrus.com/cn/pubs/proDatasheet/CS43L22_F2.pdf

**Video**

https://www.youtube.com/watch?v=2tkpF6-uGa8&feature=youtu.be

# My Project 'The Position of You'
This project is to make use of **3-axis accelerometer**, **digital microphone** and **speaker driver** of [STM32DISCOVERYF4](http://www.st.com/en/evaluation-tools/stm32f4discovery.html) board to make **"The Position of You"**. In 8 directions of tilt is the way that  you can move. When you select the way, you will press push button to select that way. (Number of movement depend on 'input value'.) You can speak loudly to microphine for reset all select ways. When you finish, board will play sound 3 time and show result.

> [View Code](/Project/)

**Video**

https://youtu.be/_SPimCJG_0Q

##Reference

**STM32CubeMX Eclipse plug in for STM32 configuration and initialization C code generation**

http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stsw-stm32095.html

**To install STM32CubeMX as an Eclipse IDE plug-in:**

http://www.openstm32.org/forumthread2046

## Developed by

[**5730625221 Mr.Sitthichai Saejia**](https://github.com/tueytoma)

**Thank you** "CP41 femily"

## About

Final Project of "HARDWARE SYNTHESIS LABORATORY I" 2016/1.

Computer Engineering CP41.

Faculty of Engineering, Chulalongkorn University.



