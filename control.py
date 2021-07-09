from pynput.keyboard import Key, Controller
import time
keyboard = Controller()
time.sleep(3)
print("start")
start = time.time()

while(time.time()-start < 3):
    keyboard.press(Key.left)
    keyboard.release(Key.left)
