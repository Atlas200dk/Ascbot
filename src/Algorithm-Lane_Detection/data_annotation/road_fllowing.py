import cv2
import os
import numpy as np
import copy
import sys

# landmark = []
img = np.zeros((1280, 720, 3), np.uint8)
img_copy = np.zeros((1280, 720, 3), np.uint8)
landmark = []
image_buffer = []
image_temporary = []

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    global img_copy, landmark
    img_copy_h, img_copy_w, _ = np.shape(img_copy)
    x_start = int(img_copy_w / 2)
    y_start = img_copy_h
    cv2.line(img_copy, (int(0), int(img_copy_h / 2)), (int(img_copy_w), int(img_copy_h / 2)), (0, 0, 255), 1)
    cv2.line(img_copy, (int(0), int(img_copy_h / 2) + int(img_copy_h / 6)),
             (int(img_copy_w), int(img_copy_h / 2) + int(img_copy_h / 6)), (0, 0, 255), 1)
    cv2.line(img_copy, (int(0), int(img_copy_h / 2) + int(img_copy_h / 3)),
             (int(img_copy_w), int(img_copy_h / 2) + int(img_copy_h / 3)), (0, 0, 255), 1)
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.line(img_copy, (x_start, y_start), (x, y),(0, 0, 255), 1)
        cv2.circle(img_copy, (x, y), 3, (0, 0, 255), -1)
        landmark.append(x)
        landmark.append(y)
    if event == cv2.EVENT_MBUTTONDOWN:
        img_copy = copy.deepcopy(img)
        print len(landmark)
        landmark.pop()
        landmark.pop()
        print len(landmark)
        for i in range(0, len(landmark), 2):
            cv2.line(img_copy, (x_start, y_start), (landmark[i], landmark[i + 1]),
                     (0, 0, 255), 1)
            cv2.circle(img_copy, (landmark[i], landmark[i + 1]), 3, (0, 0, 255), -1)
    cv2.imshow('frame',img_copy)
# frame_path = '/home/yaojt/workspaces/data_annotation/data/frame1'

def delete_txt(frame_list):
    frame_txt = []
    for frame in frame_list:
        if frame.split('.')[1] == 'txt':
            frame_txt.append(frame)
            frame_txt.append(frame.split('.')[0] + '.jpg')
    for frame_tmp in frame_txt:
        frame_list.remove(frame_tmp)
    return frame_list

def generate_framelist(frame_path):
    if os.path.exists(frame_path) is False:
        print 'the frame_path is not exits'
        sys.exit()
    frame_list = os.listdir(frame_path)
    frame_list = delete_txt(frame_list)
    return frame_list

def sort_landmark(landmark):
    (x1, y1) = (landmark[0], landmark[1])
    (x2, y2) = (landmark[2], landmark[3])
    (x3, y3) = (landmark[4], landmark[5])
    if y1 > y2:
        (x_tmp, y_tmp) = (x1, y1)
        (x1, y1) = (x2, y2)
        (x2, y2) = (x_tmp, y_tmp)
    if y1 > y3:
        (x_tmp, y_tmp) = (x1, y1)
        (x1, y1) = (x3, y3)
        (x3, y3) = (x_tmp, y_tmp)
    if y2 > y3:
        (x_tmp, y_tmp) = (x2, y2)
        (x2, y2) = (x3, y3)
        (x3, y3) = (x_tmp, y_tmp)
    landmark_sort = []
    landmark_sort.append(x1)
    landmark_sort.append(y1)
    landmark_sort.append(x2)
    landmark_sort.append(y2)
    landmark_sort.append(x3)
    landmark_sort.append(y3)
    return landmark_sort



def data_annotation(frame_path, frame_name, reflect, vission_loss):
    global img, img_copy, landmark, image_buffer, image_temporary
    img = cv2.imread(frame_path + '/' + frame_name)
    w, h, c = img.shape
    print frame_name
    # img = cv2.resize(img, (1280, 480))
    img_copy = copy.deepcopy(img)
    frame_name = frame_name.split('.')[0]
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', on_EVENT_LBUTTONDOWN)
    cv2.imshow('frame', img_copy)
    key = cv2.waitKey()
    if key == ord('s'):
        if len(landmark) == 6:
            landmarks = sort_landmark(landmark)
            landmarks = str(landmarks)
            with open(frame_path + '/' + frame_name + '.txt', 'w') as f:
                f.write(landmarks)
                f.close()
        landmark = []
        image_buffer.append(frame_name + '.jpg')
        if len(image_buffer) > 10:
            image_buffer.pop(0)
        flag = 'next'
        return flag
    if key == ord('l'):
        reflect.write('\n' + frame_name + '.jpg')
        if len(landmark) == 6:
            landmarks = sort_landmark(landmark)
            landmarks = str(landmarks)
            with open(frame_path + '/' + frame_name + '.txt', 'w') as f:
                f.write(landmarks)
                f.close()
        landmark = []
        image_buffer.append(frame_name + '.jpg')
        if len(image_buffer) > 10:
            image_buffer.pop(0)
        flag = 'next'
        return flag
    if key == ord('v'):
        vission_loss.write('\n' + frame_name + '.jpg')
        if len(landmark) == 6:
            landmarks = sort_landmark(landmark)
            landmarks = str(landmarks)
            with open(frame_path + '/' + frame_name + '.txt', 'w') as f:
                f.write(landmarks)
                f.close()
        landmark = []
        image_buffer.append(frame_name + '.jpg')
        if len(image_buffer) > 10:
            image_buffer.pop(0)
        flag = 'next'
        return flag
    if key == ord('n'):
        flag = 'next'
        return flag
    if key == ord('f'):
        flag = 'forward'
        image_temporary.append(frame_name + '.jpg')
        return flag
    if key == ord('q'):
        sys.exit()
    cv2.destroyAllWindows()
    flag = 'next'
    return flag

def generate_dataset(frame_list, frame_path, reflect, vision_loss):
    global image_buffer, image_temporary
    error_flag = 'next'
    while(len(frame_list) > 0):
        if error_flag == 'next' and len(image_temporary) == 0:
            frame = frame_list.pop()
        if error_flag == 'next' and len(image_temporary) == 1:
            frame = image_temporary[0]
            image_temporary = []
        if error_flag == 'forward':
            frame = image_buffer.pop()
        error_flag = data_annotation(frame_path, frame, reflect, vision_loss)

def main():
    assert(len(sys.argv)==2)
    frame_path = str(sys.argv[1])
    reflect = open(frame_path + '-reflect.txt', 'a')
    vision_loss = open(frame_path + '-vision_loss.txt', 'a')
    frame_list = generate_framelist(frame_path)
    generate_dataset(frame_list, frame_path, reflect, vision_loss)
    reflect.close()
    vision_loss.close()
if __name__ == '__main__':
    main()
