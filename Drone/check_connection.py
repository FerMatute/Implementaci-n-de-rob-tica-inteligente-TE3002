import logging
from djitellopy import Tello

Tello.LOGGER.setLevel(logging.DEBUG)

tello = Tello()
tello.connect()
print(tello.get_battery())