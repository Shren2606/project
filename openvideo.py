import cv2
import imutils
def main():
    cap = cv2.VideoCapture('test_video.mp4')
    

    while True:
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=600)
        if not ret:
            break

        cv2.imshow("Video", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

main()
