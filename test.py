import cv2
import droneapi

DRONE_CONNECTION_STRING = 'tcp:127.0.0.1:5762'
WAIT_READY = False
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
print('Connecting to drone on: ', DRONE_CONNECTION_STRING)
drone = droneapi.Drone(DRONE_CONNECTION_STRING, wait_ready=WAIT_READY)
drone.takeoff(5)
object_ahead = False


count =0
while count != 10:
    drone.move(droneapi.DRONE_MOVE_FORWARD,0.5,2)
    loc = drone.get_location()
    print(loc['latitude'])
    print(drone.get_heading())
    count = count + 1
while True:
    drone.change_heading(drone.get_heading() + 5)