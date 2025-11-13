"""

TESTING, DO NOT USE

"""

import cv2
import numpy as np
import imutils
import threading
import queue
import time
import select

def follow_line():
    vid = cv2.VideoCapture(1)
    while True:
        # Capture current frame from ESP32
        ret, frame = vid.read()
        if not ret:
            print("Couldn't read frame. Killing program.")

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

        # Detect contours
        gray_threshold_contours, _ = cv2.findContours(gray_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        try:
            sorted_contours = sorted(gray_threshold_contours, key=lambda c: cv2.boundingRect(c)[3], reverse=True)

            # remaining_contours = [c for c in gray_threshold_contours if not np.all(c == bestContour)]
            # remaining_contours = tuple(item for item in gray_threshold_contours if item != bestContour)

            bestContour = sorted_contours[0] if len(sorted_contours) > 0 else None
            second_contour = sorted_contours[1] if len(sorted_contours) > 1 else None

            # print(bestContour == second_contour)
        except:
            bestContour = None;
            second_contour = None;

        
        # Take the highest (yAxis) contours. Filters out some false positives
        """
        try:
            bestContour = max(gray_threshold_contours, key=lambda c: cv2.boundingRect(c)[3])
            # gray_threshold_contours.remove(bestContour)
            # second_contour = max(gray_threshold_contours, key=lambda c: cv2.boundingRect(c)[3])
        except:
            bestContour = None
            # second_contour = None
            """
        # Get frame center
        center_of_frame = np.array([frame.shape[1] / 2, frame.shape[0] / 2], int)

        try: # Does it matter which order the double line following and the single line following are in this block with regards to error handling and robustness
            
            if bestContour is not None and second_contour is not None: # Double Line Following
                cv2.putText(frame, "Double Line: FOUND", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.drawContours(frame, [bestContour], -1, (0, 255, 0), 2)
                cv2.drawContours(frame, [second_contour], -1, (0, 255, 0), 2)

                M = cv2.moments(bestContour)
                center_of_contour = np.array([int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])], int)
                center_one = center_of_contour[0]

                M2 = cv2.moments(second_contour)
                center_of_contour_two = np.array([int(M2['m10'] / M2['m00']), int(M2['m01'] / M2['m00'])], int)
                center_two = center_of_contour_two[0]

                # Calculate the center of the two contours
                center_of_countours = (center_one + center_two) / 2

                # Calculate CTE (Cross-Track Error) between detected line and frame center
                cte_x = center_of_frame[0] - center_of_countours
                cv2.putText(frame, "Cross Track Error (x): " + str(cte_x), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
                print(f"X-axis CTE: {cte_x}")
                # sock.sendall((str(cte_x) + "\n").encode("utf-8"))
            
            if bestContour is not None: # Single Line Following
                cv2.putText(frame, "Line: FOUND", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.drawContours(frame, [bestContour, second_contour], -1, (0, 255, 0), 2)

                M = cv2.moments(bestContour)
                center_of_contour = np.array([int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])], int)

                # Calculate CTE (Cross-Track Error) between detected line and frame center
                cte_x = center_of_frame[0] - center_of_contour[0]
                cv2.putText(frame, "Cross Track Error (x): " + str(cte_x), (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
                print(f"X-axis CTE: {cte_x}")
                # sock.sendall((str(cte_x) + "\n").encode("utf-8"))

            else:
                cv2.putText(frame, "Line: NOT FOUND", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        except Exception as e:
            print(f"Exception incurred: {e}")

        # Draw the x-axis median line and frame center
        cv2.circle(frame, (center_of_frame[0], center_of_frame[1]), 7, (255, 255, 255), -1)
        cv2.line(frame, (center_of_frame[0], 0), (center_of_frame[0], frame.shape[0]), (0, 255, 255), 2)
        
        # Display the current video frame
        cv2.drawContours(frame, [bestContour, second_contour], -1, (0, 255, 0), 2)
        cv2.imshow("Current Line-Following Output", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        # Read the motor settings from the ESP32
        #res = recv_line(sock)
        #if res is not None:
        #    print(f"Motor settings: {res}")
    # sock.close()
    cv2.destroyAllWindows()

def main():
    time.sleep(5)
    # Connect to both the line following and object detection ESP32s
    # line_following_sock = socket_setup()

    # start following the line
    follow_line()

if __name__ == "__main__":
    main()

