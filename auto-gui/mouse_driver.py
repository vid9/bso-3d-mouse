import sys
import serial
import pyautogui

pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False

screen_width, screen_height = pyautogui.size()

pyautogui.moveTo(screen_width/2, screen_height/2)

def mouse_click_event_handler(data):
    mouse_button, mouse_button_event = data.split(':')

    button = mouse_button.split('_')[0]

    if mouse_button_event == 'down':
        pyautogui.mouseDown(button=button)
    else:
        pyautogui.mouseUp(button=button)

pitch_mapping = screen_height / 90

yaw_mapping = screen_width / 90

def position_event_handler(data):
    roll, pitch, yaw = data.split(':')

    x_position = screen_width/2
    y_position = screen_height/2

    x_position += float(yaw)*yaw_mapping

    if x_position < 0:
        x_position = 0
    elif x_position > screen_width:
        x_position = screen_width

    y_position += float(pitch)*pitch_mapping

    if y_position < 0:
        y_position = 0
    elif y_position > screen_height:
        y_position = screen_height

    x_position = int(x_position)
    y_position = int(y_position)

    pyautogui.moveTo(x_position, y_position)

    if float(roll) > 90:
        sys.exit(1)


ser = serial.Serial(
    port='COM4',
    baudrate=115200)

print("Connected to: " + ser.portstr)

while True:
    line = str(ser.readline())

    try:
        data = line[2:][:-5]

        event, event_data = data.split('|')
        if event == 'mouse':
            mouse_click_event_handler(event_data)
        elif event == 'position':
            position_event_handler(event_data)
    except Exception as ex:
        print("Error reading data from serial", str(ex))
