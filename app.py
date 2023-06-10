from flask import Flask, render_template, Response
import cv2
import mediapipe as mp
import numpy as np
import rospy
from std_msgs.msg import Float64

app = Flask(__name__)
cap = cv2.VideoCapture(0)
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

def calculate_angle(a, b, c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle

def publish_angles(shoulder, elbow, wrist, hip, pinky):
    pub1 = rospy.Publisher('/ule_assembly_controller/Rev1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/ule_assembly_controller/Rev3_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/ule_assembly_controller/Rev4_position_controller/command', Float64, queue_size=10)
    rospy.init_node('ule_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pi = 22 / 7

    angle1 = 0  # Default value
    angle2 = 0
    angle3 = 0

    try:
        angle1 = calculate_angle(shoulder, elbow, wrist)
        angle2 = calculate_angle(elbow, shoulder, hip)
        angle3 = 180 - calculate_angle(pinky, wrist, elbow)

        position1 = -(angle2 - 20) * (pi / 180)
        position2 = (angle1 - 90) * (pi / 180)
        position3 = angle3 * (pi / 180)

        rospy.loginfo(position1)
        rospy.loginfo(angle2)
        pub1.publish(position1)

        rospy.loginfo(position2)
        rospy.loginfo(angle1)
        pub2.publish(position2)

        rospy.loginfo(position3)
        rospy.loginfo(angle3)
        pub3.publish(position3)

        rate.sleep()
    except:
        pass

@app.route('/')
def index():
    return render_template('index.html')

def generate_frames():
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                break

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            results = pose.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            try:
                landmarks = results.pose_landmarks.landmark
                shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                            landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
                elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,
                         landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
                wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,
                         landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
                hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,
                       landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
                pinky = [landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].x,
                         landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].y]

                angle1 = calculate_angle(shoulder, elbow, wrist)
                angle2 = calculate_angle(elbow, shoulder, hip)
                angle3 = 180 - calculate_angle(pinky, wrist, elbow)

                cv2.putText(image, str(angle1),
                            tuple(np.multiply(elbow, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(angle2),
                            tuple(np.multiply(shoulder, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, str(angle3),
                            tuple(np.multiply(wrist, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

                ret, jpeg = cv2.imencode('.jpg', image)
                frame = jpeg.tobytes()

                publish_angles(shoulder, elbow, wrist, hip, pinky)

                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

            except:
                pass

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(debug=True)
