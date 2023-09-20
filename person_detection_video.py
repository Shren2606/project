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
           "sofa", "train"]

cap = cv2.VideoCapture(0)
for i in range(50):
    ret, frame = cap.read()

def person():
    fps_start_time = datetime.datetime.now()
    fps = 0
    total_frames = 0

    # Tạo thư mục để lưu các frame chứa đối tượng "person"
    os.makedirs("saved_frames", exist_ok=True)

    found_person = False  # Biến cờ

    while not found_person:
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=600)
        total_frames = total_frames + 1
        count = 0

        (H, W) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 0.007843, (W, H), 127.5)

        detector.setInput(blob)
        person_detections = detector.forward()

        for i in np.arange(0, person_detections.shape[2]):
            confidence = person_detections[0, 0, i, 2]
            if confidence > 0.7:
                idx = int(person_detections[0, 0, i, 1])

                if CLASSES[idx] != "person":
                    count +=1
                    continue

                person_box = person_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = person_box.astype("int")

                print("template: ",[startX, startY, endX, endY])

                # Lưu frame chứa đối tượng "person"
                saved_frame = frame[startY:endY, startX:endX]
                cv2.imwrite(f'data/frame_template.jpg', saved_frame)
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
                cv2.imshow("Person", frame)
                
               
                found_person = True  # Đặt biến cờ thành True
                break  # Thoát khỏi vòng lặp

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

    # Kết thúc quá trình kiểm tra person

person()

template = cv2.imread('data/frame_template.jpg')

while True:

    ret, image = cap.read()

    # Tính toán kích thước của frame
    height, width, _ = image.shape

    # Chia frame thành 3 phần theo chiều ngang
    part_width = width // 3

    # Vẽ đường dọc chia các phần
    for i in range(1, 3):
        #cv2.line(image, start_point, end_point, color, thickness) 
        #start_point (x_start,Y_start), end_point(X_end,Y_End)
        cv2.line(image, (i * part_width, 0), (i * part_width, height), (255, 0, 0), 2)

    # Xác định tọa độ (x, y) của template trong ảnh
    result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    print(max_val)

    
    # Vẽ khung cho template
    w, h = template.shape[1], template.shape[0]
    top_left = max_loc # điểm trên cùng bên trái
    top_right = (top_left[0] + w,top_left[1])# điểm trên cùng bên trái
    bottom_right = (top_left[0] + w, top_left[1] + h)# điểm dưới cùng bên phải
    bottom_left =  (top_left[0] , top_left[1] + h)# điểm dưới cùng bên phải
    tâm_x = (top_left[0] + bottom_right[0]) // 2
    tâm_y = (top_left[1] + bottom_right[1]) // 2


    template2 = image[top_left[0]:top_right[0], top_left[1]:bottom_left[1]]
    #saved_frame = frame[Y_start:endY, startX:endX]
    #cv2.imshow('template2', template2)
    #cv2.imwrite(f'saved_frames/frame_template.jpg', template2)
  

    cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
    cv2.circle(image, top_left, 1, (0, 0, 255), 3) 
    cv2.circle(image, top_right, 1, (0, 0, 255), 3) 
    cv2.circle(image, bottom_right, 1, (0, 0, 255), 3)
    cv2.circle(image, bottom_left, 1, (0, 0, 255), 3)

    cv2.circle(image, (tâm_x, tâm_y), 5, (255, 255, 0), -1)# centroi

    coordinates = [top_left, top_right, bottom_right, bottom_left]
    #print("Coordinates Array:", coordinates)

    # So sánh tọa độ của template với các đường chia
    if max_val > 0.5:
        
        if tâm_x < part_width:
            print("Template thuộc phần đầu")
            cv2.putText(image, "1", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4, cv2.LINE_AA)
        elif part_width <= tâm_x < 2 * part_width:
            print("Template thuộc phần giữa")
            cv2.putText(image, "2", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4, cv2.LINE_AA)
        else:
            print("Template thuộc phần cuối")
            cv2.putText(image, "3", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4, cv2.LINE_AA)
    else:
        cv2.putText(image, "NoThing", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4, cv2.LINE_AA)

    cv2.imshow('Result', image)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
