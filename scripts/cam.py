import cv2

cam = cv2.VideoCapture(2)

while(True):
    ret, frame = cam.read()

    # print(ret)
    # print(frame)

    cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()
