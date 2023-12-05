import cv2
import time
import imutils
import numpy as np
import serial
def detection_person():
    found_person = False
    while not found_person:
        ret, image = cap.read()
        image = cv2.rotate(image,cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = imutils.resize(image, width=600)
        (H, W) = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 0.007843, (W, H), 127.5)
        detector.setInput(blob)
        person_detections = detector.forward()
        for i in np.arange(0, person_detections.shape[2]):
            confidence = person_detections[0, 0, i, 2]
            if confidence > 0.8:
                idx = int(person_detections[0, 0, i, 1])
                if CLASSES[idx] != "person":
                    continue
                person_box = person_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = person_box.astype("int")
                saved_frame = image[startY:endY, startX:endX]
                cv2.imwrite(f'data/frame_template.jpg', saved_frame)
                cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
                cv2.imshow("detection_person", image)
                found_person = True
                break
        cv2.imshow("detection_person", image)
        if cv2.waitKey(1) == ord('q'):
            break
def main():
    frames = 0
    start_time = time.time()
    template = cv2.imread('data/frame_template.jpg')
    template = imutils.resize(template, width=100)
    while True:
        ret, image = cap.read()
        #s = ser.readline()
        #print(s)
        if not ret:
            print("Không thể đọc video")
            break
        image = cv2.rotate(image,cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = imutils.resize(image, width=300)
        frames += 1
        height, width, _ = image.shape
        part_width = width // 3

        cv2.line(image, (1 * part_width, 0), (1 * part_width, height), (255, 0, 0), 2)
        cv2.line(image, (2 * part_width, 0), (2* part_width, height), (255, 0, 0), 2)

        result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc  = cv2.minMaxLoc(result)
        w, h = template.shape[1], template.shape[0]
        top_left = max_loc
        end_time = time.time()
        elapsed_time = end_time - start_time
        fps = frames / elapsed_time
        
        if max_val > 0.4:
            bottom_right = (top_left[0] + w, top_left[1] + h)
            tâm_x = (top_left[0] + bottom_right[0]) // 2
            tâm_y = (top_left[1] + bottom_right[1]) // 2
            cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
            cv2.circle(image, (tâm_x, tâm_y), 5, (255, 255, 0), -1)
            pre_tam_x= tâm_x
            pre_tam_y = tâm_y
            if tâm_x < part_width:
                print("Template thuộc phần đầu"+f"FPS: {fps:.2f}")
                cv2.putText(image, "1", (50, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1 )
                data = 100
                dataSend = f"{data}\n"
                ser.write(dataSend.encode())
            elif part_width<= tâm_x < 2 * part_width:
                print("Template thuộc phần giữa"+f"FPS: {fps:.2f}")
                cv2.putText(image, "2", (50, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)
                data = 200
                dataSend = f"{data}\n"
                ser.write(dataSend.encode())
            else:
                print("Template thuộc phần cuối"+f"FPS: {fps:.2f}")
                cv2.putText(image, "3", (50, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)
                data = 300
                dataSend = f"{data}\n"
                ser.write(dataSend.encode())
        else:
            cv2.circle(image, (pre_tam_x, pre_tam_y), 5, (0, 0, 255), -1)
            cv2.putText(image, "NoThing", (50, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)
            data = 0
            dataSend = f"{data}\n"
            ser.write(dataSend.encode())
        cv2.putText(image, f"FPS: {fps:.2f}", (50, 50),  cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 1)
        
           
        image = imutils.resize(image, width=600)
        cv2.imshow("person", image)
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    cap = cv2.VideoCapture(3)
    ser =serial.Serial(port='/dev/ttyUSB0',baudrate = 9600,timeout = 5)
    protopath = "MobileNetSSD_deploy.prototxt"
    modelpath = "MobileNetSSD_deploy.caffemodel"
    detector = cv2.dnn.readNetFromCaffe(prototxt=protopath, caffeModel=modelpath)
    CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person"]
    for i in range(100):
        ret, image = cap.read()
        if not ret:
            print("Không thể đọc video")
            break
        print(f"đọc video {i}")
    detection_person()
    main()
