**Snake in Verilog**

Well, the project implements the "Snake" game in Verilog to run on SLG47921V chips. Without apples, alas.

The snake itself is a trail of lit LEDs on 1088AS 8x8 LED matrix, driven by MAX7219 display driver. 
Its motion is controlled via an external HW-504 joystick, whose analog output is first digitalized and scaled down to 4-bit integer on Arduino UNO.

At the heart of the set up is SLG47921V FPGA chip, that reads joystick input from the Uno, computes the next frame of the game and tells MAX7219 what to draw via SPI. 

Prior to the start of the game, the chip loads a bitmap-image from BRAM and sends it to the matrix.

![20250829_184949](https://github.com/user-attachments/assets/d59582f3-8657-4f3d-8ad6-d9b4ab80c880)
