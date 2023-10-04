from flask import Flask, Response
import numpy as np
import cv2
import asyncio
import serial




async def get_max_area(rectangles):
    if len(rectangles) <= 0:
        return None
    max_area = 0
    max_rect = None
    for (x, y, w, h) in rectangles:
        if (w * h) > max_area:
            max_area = w * h
            max_rect = (x, y, w, h)
    return max_rect


async def run_face_detecion():
    # initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # alternative face detection algorithm
    face_classifier = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

    # the output will be written to output.avi
    out = cv2.VideoWriter(
        'output.avi',
        cv2.VideoWriter_fourcc(*'MJPG'),
        15.,
        (1280, 720))

    midpoint = 1280 / 2
    threshold = 20

    cv2.startWindowThread()
    cap = cv2.VideoCapture(0)

    prevRects = []

    # define direction
    dir = "right"
    # define speed
    speed = 10
    # define speed range
    speeds = [1, 5]

    custom_speed = speed


    # assume we use serial
    use_serial = True

    # define serial port
    try:
        ser = serial.Serial('/dev/cu.usbmodem11402', 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
    except:
        print("Serial Device not found, defaulting to terminal print")
        ser = "acab"
        use_serial = False



    while(True):
        # On new frame, set the movement limiter to false
        limit_movement = False
        # Capture frame-by-frame
        ret, frame = cap.read()

        # resizing for faster detection
        frame = cv2.resize(frame, (1280, 720))
        # using a greyscale picture, also for faster detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # detect people in the image
        # returns the bounding boxes for the detected objects
        # boxes, weights = hog.detectMultiScale(gray, winStride=(8,8) )

        # boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        face = face_classifier.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
        )

        faces = np.array([[x, y, x + w, y + h] for (x, y, w, h) in face])

        if len(faces) <= 0 and len(prevRects) > 0:
            faces = prevRects
            limit_movement = True
        
        for (xA, yA, xB, yB) in faces:
            # display the detected boxes in the colour picture
            cv2.rectangle(gray, (xA, yA), (xB, yB),
                            (0, 255, 0), 2)
            prevRects = faces
        
        if not limit_movement:
            
            max_rect = await get_max_area(faces)
            custom_speed = speed
            if max_rect is not None:
                range = speeds[1] - speeds[0]

                max_rect_mid = (max_rect[0] + max_rect[2]) / 2
                if max_rect_mid < midpoint - threshold:
                    dir = "left"
                    custom_speed = int((max_rect_mid / midpoint) * range) + speeds[0]
                    

                elif max_rect_mid > midpoint + threshold:
                    dir = "right"
                    custom_speed = speeds[1] - (int(((max_rect_mid - midpoint) / midpoint) * range)) + speeds[0]
                else:
                    dir = "shoot"
                    custom_speed = 0

        # Write the output video 
        out.write(gray)
        # Display the resulting frame
        cv2.imshow('test',gray)

        await handle_movement(dir, custom_speed, ser, use_serial)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    # and release the output
    out.release()
    # finally, close the window
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    if use_serial:
        await send_communications('s'.encode("ascii"), ser)

async def handle_movement(dir, speed, ser, use_serial):
    right_char = 'r' + str(speed) #.encode("ascii")
    new_line = '\n'# .encode("ascii")
    shoot_char = 's' + str(speed)# .encode("ascii")
    left_char = 'l' + str(speed)# .encode("ascii")

    if "right" in dir:
        # do whatever we need to do here
        print("move right by " + str(speed))
        if use_serial:
            await send_communications(right_char.encode("ascii"), ser)
        else:
            print(right_char.format(speed=speed).encode("ascii"))
    if "shoot" in dir:
        # do whatever we need to do here
        print("shoot")
        if use_serial:
            await send_communications(shoot_char.encode("ascii"), ser)
        else:
            print(shoot_char.format(speed=speed).encode("ascii"))
    if "left" in dir:
        # do whatever we need to do here
        print("move left by " + str(speed))
        if use_serial:
            await send_communications(left_char.encode("ascii"), ser)
        else:
            print(left_char.format(speed=speed).encode("ascii"))

    

async def send_communications(character, ser):
    ser.write(character)


# run_face_detecion()
asyncio.run(run_face_detecion())

