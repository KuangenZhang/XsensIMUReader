import keyboard
import time

while True:
    if keyboard.is_pressed('esc'):
        print('Pressed ESC')
        break
    else:
        time.sleep(1e-3)
