import cv2
from core.detection import *

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Did not receive valid frame")
        break

    edges = detect_edges(frame, "sobel", ksize=3, low=50)
    squares = detect_squares(detect_contours(edges), area_thresh=500, fill_thresh=0.4)

    if squares:
        warp = solve_homography(squares[0], frame)
        cv2.imshow('Homography', warp)

    cv2.drawContours(frame, squares, -1, (0, 255, 0), 2)
    cv2.imshow('Camera Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
