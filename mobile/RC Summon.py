import ui
import requests
import location
import time


ESP8266_IP = 'http://192.168.137.42'


def send_command(command):
    try:
        if command == 'summon':
            pos = calculateRelativePosition()
            response = requests.get(f"{ESP8266_IP}/{command}")
            print(response.text)
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")


def getCurrentLatAndLong():
    location.start_updates()
    time.sleep(3)
    current_location=location.get_location()
    latitude=current_location['latitude']
    longitude=current_location['longitude']
    location.stop_updates()
    return (latitude,longitude)
    
def calculateRelativePosition():
    latLong=getCurrentLatAndLong()
    label.text=f"{latLong}"
    
class RCCarController(ui.View):
    def __init__(self):
        self.name = 'RC Car Control'
        self.background_color = 'white'
        self.create_buttons()


    def create_buttons(self):
        # Define button properties
        buttons = [
            {'title': '↑', 'action': lambda sender: send_command('forward'), 'frame': (110, 20, 100, 50)},
            {'title': '←', 'action': lambda sender: send_command('left'), 'frame': (20, 80, 100, 50)},
            {'title': 'Stop', 'action': lambda sender: send_command('stop'), 'frame': (110, 80, 100, 50)},
            {'title': '→', 'action': lambda sender: send_command('right'), 'frame': (200, 80, 100, 50)},
            {'title': '↓', 'action': lambda sender: send_command('backward'), 'frame': (110, 140, 100, 50)},
            {'title': 'Summon', 'action': lambda sender: send_command('summon'), 'frame': (110, 200, 100, 50)},
        ]


        # Create and add buttons to the view
        for btn in buttons:
            b = ui.Button(title=btn['title'])
            b.frame = btn['frame']
            b.background_color = '#007AFF'
            b.tint_color = 'white'
            b.corner_radius = 5
            b.action = btn['action']
            self.add_subview(b)




# Present the view
view = RCCarController()
label=ui.Label()
label.frame=(60,250,200,50)
label.text=''
label.number_of_lines=0
view.add_subview(label)
view.present('sheet')

