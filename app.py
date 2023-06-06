from flask import Flask, Response, render_template
import cv2
import mediapipe as mp
import time

app = Flask(__name__)

# initialize the video stream
vs = cv2.VideoCapture(0)

mpDraw = mp.solutions.drawing_utils
mpPose = mp.solutions.pose
pose = mpPose.Pose()

@app.route("/")
def index():
    return render_template('index.html')

def detect_pose():
    global vs, mpDraw, mpPose, pose

    while True:
        ret, frame = vs.read()

        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(imgRGB)

        if results.pose_landmarks:
            mpDraw.draw_landmarks(frame, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

        (flag, encodedImage) = cv2.imencode(".jpg", frame)

        if not flag:
            continue

        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

@app.route("/video_feed")
def video_feed():
    return Response(detect_pose(), mimetype = "multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, threaded=True, port="5001")
