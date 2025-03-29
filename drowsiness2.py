import cv2
import dlib
import pyttsx3
import serial
from scipy.spatial import distance
import time

# Text-to-Speech
engine = pyttsx3.init()

# Initialize webcam
cap = cv2.VideoCapture(0)

# Face detector and landmark predictor
face_detector = dlib.get_frontal_face_detector()
dlib_facelandmark = dlib.shape_predictor(r"C:\Users\avish\OneDrive\Desktop\CLASS NOTES\SEMESTER - 6\Engineering Clinics - II\shape_predictor_68_face_landmarks.dat")

# Serial connection to Arduino
try:
    arduino = serial.Serial(port='COM3', baudrate=9600, timeout=3)
    time.sleep(2)
except serial.SerialException:
    print("⚠ Arduino not connected.")
    arduino = None

def detect_EAR(eye_points):
    return (distance.euclidean(eye_points[1], eye_points[5]) + distance.euclidean(eye_points[2], eye_points[4])) / (2 * distance.euclidean(eye_points[0], eye_points[3]))

DROWSY_FRAMES = 90
blink_counter = 0
drowsy_state = False  # Track state so we don't send 'D' repeatedly

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_detector(gray)

    for face in faces:
        landmarks = dlib_facelandmark(gray, face)
        left_eye = detect_EAR([(landmarks.part(n).x, landmarks.part(n).y) for n in range(36, 42)])
        right_eye = detect_EAR([(landmarks.part(n).x, landmarks.part(n).y) for n in range(42, 48)])
        ear = (left_eye + right_eye) / 2

        if ear < 0.25:
            blink_counter += 1
        else:
            blink_counter = 0
            if drowsy_state:  # If previously drowsy, now back to normal
                if arduino:
                    arduino.write(b'N')
                drowsy_state = False

        if blink_counter >= DROWSY_FRAMES and not drowsy_state:
            print("[!] Drowsiness Detected")
            engine.say("Wake up!")
            engine.runAndWait()
            if arduino:
                arduino.write(b'D')
                arduino.flush()
                time.sleep(0.1)
            drowsy_state = True

    cv2.imshow("Drowsiness Detection", frame)
    if cv2.waitKey(9) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()