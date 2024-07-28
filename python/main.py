from time import sleep

import cv2
import mediapipe as mp
import serial
from datetime import datetime

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils


keypoint_drawing_spec = mp_drawing.DrawingSpec(color=(255, 255, 255), circle_radius=1)
bbox_drawing_spec = mp_drawing.DrawingSpec(color=(9, 255, 0), thickness=2)


class Webcam:
    def __init__(self):
        self.attempts = 5
        self.seconds_between_attempts = 10

        self.port = "/dev/tty.usbmodem2201"
        self.baud_rate = 9600

        self.servo_position = 90

        self.cap = self.connect_to_webcam()
        self.arduino = self.connect_to_arduino()

    def connect_to_webcam(self) -> cv2.VideoCapture:
        cap = None
        for i in range(1, self.attempts):
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print(
                    f"Webcam not found, trying again in {self.seconds_between_attempts} seconds... "
                    f"{self.attempts - i} attempts left."
                )
                sleep(self.seconds_between_attempts)
                continue

        if not cap.isOpened():
            exit("Process has been terminated.")

        return cap

    def connect_to_arduino(self) -> serial.Serial:
        arduino = serial.Serial(self.port, self.baud_rate)
        return arduino

    def run(self) -> None:
        with mp_face_detection.FaceDetection(
            model_selection=0, min_detection_confidence=0.75
        ) as face_detection:
            while self.cap.isOpened():
                ret, frame = self.cap.read()
                if not ret:
                    break

                # Each frame is resized to 1/4 size for faster face recognition processing
                resized_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                rgb_resized_frame = resized_frame[:, :, ::-1]

                # Draw a rectangle showing the focus area
                ih, iw, _ = resized_frame.shape

                area_width = iw // 3
                area_height = ih

                area_x1 = (iw - area_width) // 2
                area_y1 = 0

                area_x2 = area_x1 + area_width
                area_y2 = ih

                cv2.rectangle(resized_frame, (area_x1, area_y1), (area_x2, area_y2), (0, 255, 255), 2)

                # Look for faces
                results = face_detection.process(rgb_resized_frame)

                if results.detections:
                    print('A FACE HAS BEEN FOUND')
                    # Sort the detections list so the closest (or biggest) face is the first one
                    results.detections.sort(
                        key=lambda det: det.location_data.relative_bounding_box.width
                        * det.location_data.relative_bounding_box.height,
                        reverse=True,
                    )

                    #  Only process one detection
                    detection = results.detections[0]

                    #  Get the bounding box coordinates
                    bbox = detection.location_data.relative_bounding_box
                    x, y, w, h = (
                        int(bbox.xmin * iw),
                        int(bbox.ymin * ih),
                        int(bbox.width * iw),
                        int(bbox.height * ih),
                    )
                    center_x, center_y = x + w // 2, y + h // 2

                    #  Add a circle to show the center of the face
                    cv2.circle(
                        resized_frame,
                        (center_x, center_y),
                        keypoint_drawing_spec.thickness,
                        keypoint_drawing_spec.color,
                        -1,
                    )

                    # Check if the servo has to be moved
                    new_position = self.servo_position
                    if center_x <= area_x1 and new_position + 10 <= 180:
                        new_position += 10
                    elif center_x >= area_x2 and new_position - 10 >= 0:
                        new_position -= 10

                    if self.servo_position != new_position:
                        self.arduino.write(f'{new_position}\n'.encode())
                        self.servo_position = new_position
                        sleep(0.5)

                else:
                    print('NO FACE FOUND')

                cv2.imshow("Webcam Feed", resized_frame)

                if cv2.waitKey(1) and 0xFF == ord("q"):
                    break


if __name__ == "__main__":
    webcam = Webcam()
    webcam.run()

    cv2.destroyAllWindows()
