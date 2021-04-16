import cv2
import time


tracker = cv2.TrackerGOTURN_create()
initBB = None

# vs = cv2.VideoCapture("walking.mp4")
vs = cv2.VideoCapture(0)
while True:
    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    ret, frame = vs.read()
    if frame is None:
        break
    # resize the frame (so we can process it faster) and grab the
    # frame dimensions
    frame = cv2.resize(frame, (frame.shape[0], 700))
    (H, W) = frame.shape[:2]
    if initBB is not None:
        (success, box) = tracker.update(frame)
        # check to see if the tracking was a success
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          (0, 255, 0), 2)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("s"):
        initBB = cv2.selectROI("Frame", frame, fromCenter=False,
                               showCrosshair=True)
        print("InitBB {}".format(initBB))

        tracker.init(frame, initBB)

        # if the `q` key was pressed, break from the loop
    elif key == ord("q"):
        break
    # time.sleep(0.1)
cv2.destroyAllWindows()

