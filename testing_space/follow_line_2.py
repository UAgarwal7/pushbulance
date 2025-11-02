"""
I'm testing cv2 with the mac built-in camera to see what the code does and whatnot

"""

import cv2
import numpy as np
import imutils
import threading
import queue
import time
import select

low_black = np.uint8([55, 55, 55])
high_black = np.uint8([0, 0, 0])
kernel = np.ones((15, 15), np.float32) / 255

def follow_line(cap):
    while True:
        # Capture current frame from ESP32
        ret, frame = cap.read()
        if not ret:
            print("Couldn't read frame. Killing program.")
            exit()

        
        # Blur the frame to reduce noise
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        

        # Get black pixels (these values depend on room light)
        gray_threshold = cv2.adaptiveThreshold(
            gray_frame,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            103,  # block size
            10   # constant to subtract
        )
        # Fill small holes in detection
        hole_kernel = np.ones((5, 5), np.uint8)
        gray_threshold = cv2.morphologyEx(gray_threshold, cv2.MORPH_CLOSE, hole_kernel)

        """
        # Detect contours
        gray_threshold_contours, _ = cv2.findContours(gray_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Take the highest (yAxis) contours. Filters out some false positives
        try:
            bestContour = max(gray_threshold_contours, key=lambda c: cv2.boundingRect(c)[3])
        except:
            bestContour = None

        # Get frame center
        center_of_frame = np.array([frame.shape[1] / 2, frame.shape[0] / 2], int)

        try:
            if bestContour is not None:
                cv2.putText(frame, "Line: FOUND", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.drawContours(frame, [bestContour], -1, (0, 255, 0), 2)

                M = cv2.moments(bestContour)
                center_of_contour = np.array([int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])], int)

                # Calculate CTE (Cross-Track Error) between detected line and frame center
                cte_x = center_of_frame[0] - center_of_contour[0]
                cv2.putText(frame, "Cross Track Error (x): " + str(cte_x), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
                print(f"X-axis CTE: {cte_x}")
                sock.sendall((str(cte_x) + "\n").encode("utf-8"))

            else:
                cv2.putText(frame, "Line: NOT FOUND", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        except Exception as e:
            print(f"Exception incurred: {e}")

        # Draw the x-axis median line and frame center
        cv2.circle(frame, (center_of_frame[0], center_of_frame[1]), 7, (255, 255, 255), -1)
        cv2.line(frame, (center_of_frame[0], 0), (center_of_frame[0], frame.shape[0]), (0, 255, 255), 2)
        

        # Display the current video frame
        cv2.imshow("Current Line-Following Output", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        # Read the motor settings from the ESP32
        #res = recv_line(sock)
        #if res is not None:
        #    print(f"Motor settings: {res}")
        """
        # cv2.line(frame, (50,50), (100,100), (0,0,0), 5)
        cv2.imshow('Live feed', gray_frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

def main():
    time.sleep(5)
    # Connect to both the line following and object detection ESP32s
    cap = cv2.VideoCapture(1)

    # start following the line
    follow_line(cap)

if __name__ == "__main__":
    main()