import cv2

cap = cv2.VideoCapture(0)

# Lấy frame background
for i in range(10):
    ret, frame = cap.read()

template = cv2.imread('template.jpg')

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
    top_left = max_loc # điểm trên cùng bên trái
    w, h = template.shape[1], template.shape[0]
    bottom_right = (top_left[0] + w, top_left[1] + h)# điểm dưới cùng bên phải
    tâm_x = (top_left[0] + bottom_right[0]) // 2
    tâm_y = (top_left[1] + bottom_right[1]) // 2

    cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
    cv2.circle(image, max_loc, 1, (0, 0, 255), 3)
    cv2.circle(image, bottom_right, 1, (0, 0, 255), 3)
    cv2.circle(image, (tâm_x, tâm_y), 5, (255, 255, 0), -1)

    # So sánh tọa độ của template với các đường chia
    if max_val > 0.6:
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
