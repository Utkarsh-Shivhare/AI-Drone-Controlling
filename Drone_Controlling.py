from __future__ import print_function
#from imutils.video.pivideostream import PiVideoStream
from pivideo import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=1000,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())
connection_string = '/dev/serial0'  # Update this with your actual serial port
baud_rate = 921600  # Update this with your desired baud rate
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string,baud=baud_rate)

# initialize the camera and stream
camera = PiCamera()
camera.resolution = (320, 320)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(320, 320))
stream = camera.capture_continuous(rawCapture, format="bgr",
	use_video_port=True)
print("[INFO] sampling frames from `picamera` module...")
time.sleep(2.0)
fps = FPS().start()
# loop over some frames
for (i, f) in enumerate(stream):
	# grab the frame from the stream and resize it to have a maximum
	# width of 400 pixels
	frame = f.array
	frame = imutils.resize(frame, width=320)
	
	# check to see if the frame should be displayed to our screen
	if args["display"] > 0:
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame and update
	# the FPS counter
	rawCapture.truncate(0)
	fps.update()
	# check to see if the desired number of frames have been reached
	if i == 100:
		break
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# do a bit of cleanup
cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()
net = cv2.dnn.readNet("yolov4-tiny-custom_2000.weights", "yolov4-tiny-custom.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Load class names
with open("obj.names", "r") as f:
    classes = f.read().strip().split("\n")

print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()
# loop over some frames...this time using the threaded stream
while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=320)
    image = cv2.resize(frame, (320, 320))
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (320, 320), swapRB=True, crop=False)
    net.setInput(blob)
    detections = net.forward(output_layers)
    for out in detections:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                center_x = int(detection[0] * image.shape[1])
                center_y = int(detection[1] * image.shape[0])
                w = int(detection[2] * image.shape[1])
                h = int(detection[3] * image.shape[0])

                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label = f"{classes[class_id==0]}: {confidence:.2f}"
                cv2.putText(image, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                if classes[class_id==0]=="hotspot":
                    print("hotspot detected")
                    servo_pin=8
                    servo_pwm_value = 2000
                    vehicle.channels.overrides[8] = servo_pwm_value
			
    if True:
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
    #rawCapture.truncate(0)
    fps.update()
    if key == ord("q"):
        break
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
