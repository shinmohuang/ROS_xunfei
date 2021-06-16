
import cv2
import numpy as np
import cv2.aruco as aruco

cap = cv2.VideoCapture(0)
while(1):
    # get a frame
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)   ##图像左右颠倒
    # show a frame
    # cv2.imshow("capture", frame)
    img_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # print(img.shape)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    corners, ids, _ = aruco.detectMarkers(img_gray, aruco_dict)
    # frame = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # print('corners',corners)
    # print('ids', type(ids))
    aruco.drawDetectedMarkers(frame, corners, ids)
    if ids is not None:
        for i in range(len(ids)):
            print('发现泊车码：', i)
            cv2.putText(frame, 'aruco:{}'.format(i), (20, 35*(i+1)), 0, 1, [0, 255, 0], thickness=2, lineType=cv2.LINE_AA)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
    else:
        cv2.putText(frame, 'no aruco'.format(0), (20, 35), 0, 1, [0, 0, 255], thickness=2, lineType=cv2.LINE_AA)
    cv2.imshow('Detected AruCo markers', frame);'o'
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()


