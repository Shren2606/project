import cv2
def main():
    cap = cv2.VideoCapture("http://192.168.1.8:4747/video")
    while True:
        ret, image = cap.read()

        if not ret:
            print("Không thể đọc video")
            break
        cv2.imshow('Result', image)
        print(image.shape)
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
