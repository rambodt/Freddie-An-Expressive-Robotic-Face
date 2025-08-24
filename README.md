# Freddie – An Expressive Robotic Face  

## What is this?  
Freddie is a robotic mask that can show **different facial expressions** like happy, angry, sad, and more. It’s controlled wirelessly by another ESP32 with joysticks and an LCD screen.  

This project combines **FreeRTOS**, **ESP-NOW wireless**, and **servo control** to bring a face to life.  

---

## Hardware  
- 2x ESP32-S3 (mask + controller)  
- 13 servo motors (eyebrows, eyes, eyelids, jaw, mouth)  
- PCA9685 16-channel servo driver (I²C)  
- Dual joysticks + button  
- 16x2 I²C LCD
- Buzzer
- ELEGOO Breadboard Power Supply MB V2

---

## Modes  
- **Free Play** → Move servos directly with joysticks  
- **Game Mode** → Match random faces within a time limit  

---

## Demo  
 [Watch the YouTube demo](https://youtu.be/wkVzFOGhkFc)  

---

## Credits  
- Built by Rambod Taherian and Kourosh Ghahramani  
- Uses [Adafruit PCA9685 Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) and [ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)  
- The eye mechanism is based on the design by [Morgan Manly](https://makerworld.com/en/models/1217039-animatronic-eyes-compact-with-arduino#profileId-1233118) with small modifications to adapt it for this project.
