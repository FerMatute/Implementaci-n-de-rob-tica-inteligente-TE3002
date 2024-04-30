from djitellopy import Tello

SSID = 'Kamikaze'
PASSWORD = 'HITLER123'

drone = Tello()
drone.connect()
drone.set_wifi_credentials(SSID, PASSWORD)

print('Wifi config set to: ')
print('SSID: ' + SSID + 'PASSWORD: ' + PASSWORD)