from flask import Flask, render_template, Response, request
import numpy as np
import cv2
import serial

# Initialize Flask
app = Flask(__name__)

# Initialize Webcam
# Use 0 for Logitech, or iphone (kind of buggy)
camera = cv2.VideoCapture(0)  # use 0 for web camera
#  for cctv camera use rtsp://username:password@ip_address:554/user=username_password='password'_channel=channel_number_stream=0.sdp' instead of camera
# for local webcam use cv2.VideoCapture(0)

# OpenCV Face Detection Models
# We use CascadeClassifier curently because it is more reliable
# TODO: Either train a custom model OR use both models for better accuracy
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# alternative face detection algorithm
face_classifier = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)

# Set up serial
# define serial port
try:
    ser = serial.Serial('/dev/tty.usbmodem1103', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
except:
    print("Serial Device not found, defaulting to terminal print")
    ser = "NO_SERIAL_FOUND"
    use_serial = False

# Currently Unusued
# TODO: Fix this or remove
# the output will be written to output.avi
# out = cv2.VideoWriter(
#    'output.avi',
#    cv2.VideoWriter_fourcc(*'MJPG'),
#    15.,
#    (1280, 720))

# Constant Variables
# Window Variables
midpoint = 1280 / 2
threshold = 20
# Necessary stored information
# previous rectangle holder
prevRects = []

# Turret Movement Variables
dir = "right"
# Standard Speed
speed = 15
# Range of Speed
speeds = [20, 25]
# Custom Speed used by function
custom_speed = speed
use_serial = True

# TODO: Fix this?
# Currenlty not working
# Shutdown boolean for serial communication
shutdown = False

# Methods

# Get Max Area
# Takes in a list of OpenCV rectangles, and returns the largest
# Used to find the loargest "face" found by the face detection model
def get_max_area(rectangles):
    if len(rectangles) <= 0:
        return None
    max_area = 0
    max_rect = None
    for (x, y, w, h) in rectangles:
        if (w * h) > max_area:
            max_area = w * h
            max_rect = (x, y, w, h)
    return max_rect

# Runs face detection on a frame and returns the processed frame

def run_face_detecion(frame):
    global midpoint, threshold, prevRects, dir, speed, speeds, custom_speed, use_serial
    # On new frame, set the movement limiter to false
    limit_movement = False
    # Capture frame-by-frame

    # resizing for faster detection
    frame = cv2.resize(frame, (1280, 720))
    # using a greyscale picture, also for faster detection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Detect people in the images using HOG (old algo)
    # TODO: Integrate this better?
    # returns the bounding boxes for the detected objects
    # boxes, weights = hog.detectMultiScale(gray, winStride=(8,8) )
    # boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

    # Run face classifier algorithm
    face = face_classifier.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
    )
    # Use np to extract relevant information
    faces = np.array([[x, y, x + w, y + h] for (x, y, w, h) in face])

    # Determine if a face has been detected in the current frame
    # In case it has not, tell the program it can't manipulate the motor using the lim_mov bool
    # TODO: Maybe work on something better here. This can cause a face to constantly be detected even though it's not there
    if len(faces) <= 0 and len(prevRects) > 0:
        faces = prevRects
        limit_movement = True
    
    # Display the detected faces on the image
    for (xA, yA, xB, yB) in faces:
        # display the detected boxes in the colour picture
        cv2.rectangle(gray, (xA, yA), (xB, yB),
                        (0, 255, 0), 2)
        prevRects = faces
    
    # If the program has permission to manipulate the motor, determine what direction and speed should be used using this if branch
    if not limit_movement:
        # Get the max area rectangle. We assume this is the face (face detection can have false positives but these are typically small)
        max_rect = get_max_area(faces)
        # Set our custom speed to the predefined speed
        # This should not be used and is legacy code
        custom_speed = speed
        # Additional error handling for first frame, run speed algo here
        if max_rect is not None:
            # Grab the range of the predefined speeds and calculate the midpoint
            range = speeds[1] - speeds[0]
            # Calculate the midpoint of the rectangle's location
            max_rect_mid = (max_rect[0] + max_rect[2]) / 2
            # Determine where the rectangle lies
            # Lies on left
            if max_rect_mid < midpoint - threshold:
                dir = "left"
                custom_speed = int((max_rect_mid / midpoint) * range) + speeds[0]
            # Lies on right
            elif max_rect_mid > midpoint + threshold:
                dir = "right"
                custom_speed = speeds[1] - (int(((max_rect_mid - midpoint) / midpoint) * range)) + speeds[0]
            # Lies in center
            else:
                dir = "shoot"
                custom_speed = 0
        # Call the handle movement function to communicate with
        handle_movement(dir, custom_speed, ser, use_serial)
    
    # Variables for on-display text
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50, 50)
    fontScale = 1
    color = (255, 0, 0)
    thickness = 2
    # Ondisplay text
    img_text = "shoot"
    # If left or right, write the appropriate 
    if "shoot" not in dir:
        img_text = "move " + dir + " by " + str(custom_speed)

    # Put the text on the img
    final_frame = cv2.putText(gray, img_text, org, font, fontScale, color, thickness, cv2.LINE_AA)
    
    # Return the final frame to be displayed
    return final_frame

# Determines what should be sent to the device over serial, processes these outputs into a manageable format
# (ascii for chars, bytes for integers)
# TODO: Get shutdown to work properly
def handle_movement(dir, speed, ser, use_serial):
    global shutdown
    # Constant chars to be used
    right_char = 'r'
    new_line = '\n'
    shoot_char = 's'
    left_char = 'l'

    # Process the data
    # If serial not connected, we print what would be sent
    if "right" in dir:
        if use_serial:
            send_communications(right_char.encode("ascii"), ser)
            send_communications(speed.to_bytes(1, 'big'), ser)
        else:
            print(right_char.format(speed=speed).encode("ascii"))
            print(str(custom_speed))
    if "shoot" in dir:
        if use_serial:
            ten_speed = 10
            send_communications(shoot_char.encode("ascii"), ser)
            send_communications(ten_speed.to_bytes(1, 'big'), ser)
        else:
            print(shoot_char.format(speed=speed).encode("ascii"))
            print(str(custom_speed))
    if "left" in dir:
        if use_serial:
            send_communications(left_char.encode("ascii"), ser)
            send_communications(speed.to_bytes(1, 'big'), ser)
        else:
            print(left_char.format(speed=speed).encode("ascii"))
            print(str(custom_speed))

# Use pyserial to send the input to the device
def send_communications(character, ser):
    ser.write(character)

# Generates frame by frame from camera and sends to the webpage thru flask
# Runs the facial recognition, which in turn communicates with through serial
def gen_frames():  # generate frame by frame from camera
    while True:
        # Capture frame-by-frame
        success, frame = camera.read()  # read the camera frame
        if not success:
            # Camera could not take a photo, break out
            break
        else:
            # Resize frame for faster detection
            frame = cv2.resize(frame, (1280, 720))
            # Run face detection algorithm
            frame = run_face_detecion(frame)
            # Encode frame as jpeg and convert to bytes, so that it can be processed by flask
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # Concat frame one by one and show result

# Routes the video feed into the html
@app.route('/video_feed')
def video_feed():
    #Video streaming route. Put this in the src attribute of an img tag
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Sets the "homepage"
# Here it pulls from the template index.html
@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

# Shuts down the entire system when a button is clicked in html
# TODO: Make this work, currently it crashes as there is no apparent way to elegantly shut down a flask server
# TODO: Maybe just kill the program?
@app.route('/shutdown')
def shutdown():
    global ser, shutdown
    shutdown = True
    if use_serial:
        ten_speed = 10
        send_communications("s".encode("ascii"), ser)
        send_communications(ten_speed.to_bytes(1, 'big'), ser)
    print("hello world")
    return 'Server shutting down...'

# Main, simply run the flask app
if __name__ == '__main__':
    app.run(host = "192.168.1.37", debug=False)