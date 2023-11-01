# This is a sample Python script.

# Press Maiusc+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import pyvjoy

import time


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

    # Pythonic API, item-at-a-time

    j = pyvjoy.VJoyDevice(1)

    for i in range(20):
        i=i+1
        print(f"b: {i}")
        j.set_button(i, 1)
        time.sleep(1)
    # turn button number 15 on
    j.set_button(15, 1)

    # Notice the args are (buttonID,state) whereas vJoy's native API is the other way around.

    # turn button 15 off again
    j.set_button(15, 0)

    # Set X axis to fully left
    j.set_axis(pyvjoy.HID_USAGE_X, 0x1)

    # Set X axis to fully right
    j.set_axis(pyvjoy.HID_USAGE_X, 0x8000)

    # Also implemented:

    j.reset()
    j.reset_buttons()
    j.reset_povs()

    j.data.lButtons = 19  # buttons number 1,2 and 5 (1+2+16)
    j.data.wAxisX = 0x2000
    j.data.wAxisY = 0x7500

    # send data to vJoy device
    j.update()
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
