import cv2
import numpy as np
import lcm
import time
from exlcm.heartbeat_t import heartbeat_t
from exlcm.object_t import object_t

lc = lcm.LCM()


def send_message(detected):
    """Send an LCM message when motion is detected."""
    msg = object_t()
    msg.timestamp = int(time.time() * 1000000)
    msg.name = "CollisionDetector"
    msg.detectedObject = detected

    lc.publish("OBJECT", msg.encode())
    print(f"Message sent: Detected object: {detected}")

def main():
    camera = cv2.VideoCapture(0)

    background_ret, background_frame = camera.read()

    background_gray = cv2.cvtColor(background_frame, cv2.COLOR_BGR2GRAY)
    background_gray = cv2.GaussianBlur(background_gray, (21, 21), 0)

    while True:
        ret, frame = camera.read()

        if not ret:
            break

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)

        frame_delta = cv2.absdiff(background_gray, gray_frame)
        thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]

        background_gray = gray_frame

        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        motion_detected = False

        for contour in contours:
            if cv2.contourArea(contour) < 500:
                continue

            motion_detected = True

        send_message(motion_detected)

        cv2.imshow("Frame", frame)
        cv2.imshow("Thresh", thresh)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
