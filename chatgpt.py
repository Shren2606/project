import cv2
import datetime
import imutils
import numpy as np
import os

protopath = "MobileNetSSD_deploy.prototxt"
modelpath = "MobileNetSSD_deploy.caffemodel"
detector = cv2.dnn.readNetFromCaffe(prototxt=protopath, caffeModel=modelpath)

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

def main():
    cap = cv2.VideoCapture('test_video.mp4')

    fps_start_time = datetime.datetime.now()
    fps = 0
    total_frames = 0

    # Tạo thư mục để lưu các frame chứa đối tượng "person"
    os.makedirs("saved_frames", exist_ok=True)

    # Lấy frame background
    for i in range(10):
        ret, frame = cap.read()

    template = cv2.imread('template.jpg')

    while True:
        ret, image = cap.read()
        frame = imutils.resize(image, width=600)
        total_frames = total_frames + 1

        (H, W) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 0.007843, (W, H), 127.5)

        detector.setInput(blob)
        person_detections = detector.forward()

        for i in np.arange(0, person_detections.shape[2]):
            confidence = person_detections[0, 0, i, 2]
            if confidence > 0.5:
                idx = int(person_detections[0, 0, i, 1])

                if CLASSES[idx] != "person":
                    continue

                person_box = person_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = person_box.astype("int")

                # Lưu frame chứa đối tượng "person"
                saved_frame = frame[startY:endY, startX:endX]
                cv2.imwrite(f'data/frame_template.jpg', saved_frame)

                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

        fps_end_time = datetime.datetime.now()
        time_diff = fps_end_time - fps_start_time
        if time_diff.seconds == 0:
            fps = 0.0
        else:
            fps = (total_frames / time_diff.seconds)

        fps_text = "FPS: {:.2f}".format(fps)

        cv2.putText(frame, fps_text, (5, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)

        cv2.imshow("Application", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

main()