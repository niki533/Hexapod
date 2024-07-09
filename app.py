from flask import Flask, render_template, request, Response, render_template_string
import threading
import time
import test
from __future__ import division
import io
import cv2
import numpy as np
import picamera

app = Flask(__name__)

direction_command = 'no'
turn_command = 'no'
step_set = 1

def move_thread():
    global step_set, direction_command, turn_command
    while True:
        if direction_command == 'forward' and turn_command == 'no':
            test.move(step_set, 35, 'no')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        elif direction_command == 'backward' and turn_command == 'no':
            test.move(step_set, -35, 'no')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        elif turn_command == 'left':
            test.move(step_set, 35, 'left')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        elif turn_command == 'right':
            test.move(step_set, 35, 'right')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        else:
            test.stand()
            time.sleep(0.1)

def start_movement_thread():
    thread = threading.Thread(target=move_thread)
    thread.start()

def generate_frames():
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 24
        stream = io.BytesIO()
        prev_frame = None

        for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            stream.seek(0)
            frame = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if prev_frame is None:
                prev_frame = gray
                stream.seek(0)
                stream.truncate()
                continue

            frame_delta = cv2.absdiff(prev_frame, gray)
            thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)

            contours = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]

            for contour in contours:
                if cv2.contourArea(contour) < 500:
                    continue
                (x, y, w, h) = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            prev_frame = gray

            ret, jpeg = cv2.imencode('.jpg', frame)
            stream.seek(0)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            stream.seek(0)
            stream.truncate()

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/move', methods=['POST'])
def move_command():
    global direction_command, turn_command
    command = request.form['command']
    if command == 'forward':
        direction_command = 'forward'
        turn_command = 'no'
    elif command == 'backward':
        direction_command = 'backward'
        turn_command = 'no'
    elif command == 'left':
        direction_command = 'no'
        turn_command = 'left'
    elif command == 'right':
        direction_command = 'no'
        turn_command = 'right'
    elif command == 'stop':
        direction_command = 'no'
        turn_command = 'no'
        test.stand()
    return 'OK'

if __name__ == "__main__":
    start_movement_thread()
    app.run(host='0.0.0.0', port=5000, threaded=True)