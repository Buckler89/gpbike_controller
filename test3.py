from pyvjoystick import vigem as vg

gamepad = vg.VX360Gamepad()

gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)  # press the A button
gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)  # press the left hat button

gamepad.update()  # send the updated state to the computer

# (...) A and left hat are pressed...

gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)  # release the A button

gamepad.update()  # send the updated state to the computer

# (...) left hat is still pressed...


gamepad.left_trigger(value=100)  # value between 0 and 255
gamepad.right_trigger(value=255)  # value between 0 and 255
gamepad.left_joystick(x_value=-10000, y_value=0)  # values between -32768 and 32767
gamepad.right_joystick(x_value=-32768, y_value=15000)  # values between -32768 and 32767

gamepad.update()


gamepad.left_trigger_float(value_float=0.5)  # value between 0.0 and 1.0
gamepad.right_trigger_float(value_float=1.0)  # value between 0.0 and 1.0
gamepad.left_joystick_float(x_value_float=-0.5, y_value_float=0.0)  # values between -1.0 and 1.0
gamepad.right_joystick_float(x_value_float=-1.0, y_value_float=0.8)  # values between -1.0 and 1.0

gamepad.update()


gamepad.reset()

gamepad.update()