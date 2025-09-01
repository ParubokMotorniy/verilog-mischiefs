**Snake in Verilog**

Well, the project implements the "Snake" game in Verilog to run on SLG47921V chips. Without apples, alas.

The snake itself is a trail of lit LEDs on 1088AS 8x8 LED matrix, driven by MAX7219 display driver. 
Its motion is controlled via an external HW-504 joystick, whose analog output is first digitalized and scaled down to 4-bit integer on Arduino UNO.

At the heart of the set up is SLG47921V FPGA chip, that reads joystick input from the Uno, computes the next frame of the game and tells MAX7219 what to draw via SPI. 
