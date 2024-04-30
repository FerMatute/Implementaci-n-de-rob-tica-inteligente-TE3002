from djitellopy import Tello

drone = Tello()

drone.connect()
drone.query_battery()