#!/bin/python
#### in rtsp server:
####  ffserver -f /etc/ffserver.conf &
#### ffmpeg -f v4l2 -i /dev/video0  -s 640x480 -r 24 -vcodec libx264 -an http://127.0.0.1:8090/feed1.ffm
import cv2

if __name__ == "__main__":
    #ip='192.168.65.119'
    ip='192.168.31.119'
    cap = cv2.VideoCapture("rtsp://{}:8554/robot".format(ip))
    #cap = cv2.VideoCapture(0)
    while cap.isOpened():
        (ret,frame)=cap.read()
        print (frame.shape[0], frame.shape[1], frame.shape[2])
        cv2.imshow("frame",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyWindow("frame")
    cap.release()
