import cv2
import numpy as np
import sys
import os
import copy

global img_copy
landmark = []

def obtain_txt(frame_list):
    frame_txt = []
    for frame in frame_list:
        if frame.split('.')[1] == 'txt':
            frame_txt.append(frame)
    return frame_txt


def generate_framelist(frame_path):
    if os.path.exists(frame_path) is False:
        print 'the frame_path is not exits'
        sys.exit()
    frame_list = os.listdir(frame_path)
    frame_txt = obtain_txt(frame_list)
    return frame_txt

def draw_point(frame_path, img, frame_txt_name):
    with open(frame_path + '/' + frame_txt_name, 'r') as f:
        lines = f.read()
    # print lines
    # print type(lines)
    lines = lines[1:-1]
    # print lines
    lines = lines.split(',')
    # print lines
    landmark = []
    for line in lines:
        landmark.append(int(line))
    # print landmark
    cv2.circle(img, (landmark[0], landmark[1]), 3, (0, 0, 255), -1)

    cv2.circle(img, (landmark[2], landmark[3]), 3, (0, 0, 255), -1)
    cv2.circle(img, (landmark[4], landmark[5]), 3, (0, 0, 255), -1)

    f.close()

    return img
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
def modify_data_annotation(frame_path, frame_txt_name):
    print frame_txt_name
    global img, img_copy, landmark, image_buffer, image_temporary
    cv2.setMouseCallback('frame', on_EVENT_LBUTTONDOWN)
    cv2.imshow('frame', img_copy)
    key = cv2.waitKey()
    if key == ord('s'):
        if len(landmark) == 6:
            landmarks = sort_landmark(landmark)
            landmarks = str(landmarks)
            with open(frame_path + '/' + frame_txt_name, 'w') as f:
                f.write(landmarks)
                f.close()
        landmark = []
        return
        # image_buffer.append(frame_name + '.jpg')
        # if len(image_buffer) > 10:
        #     image_buffer.pop(0)
    if key == ord('n'):
        return
        # flag = 'next'
    #     return flag
    # if key == ord('n'):
    #     flag = 'next'
    #     return flag
    # if key == ord('f'):
    #     flag = 'forward'
    #     image_temporary.append(frame_name + '.jpg')
    #     return flag
    # if key == ord('q'):
    #     sys.exit()
    # cv2.destroyAllWindows()
    # flag = 'next'
    # return flag

def verify_data_annotation(frame_path, frame_name):
    global img_copy
    frame_txt_name = frame_name
    print frame_txt_name
    frame_image_name = frame_name.split('.')[0] + '.jpg'
    img = cv2.imread(frame_path + '/' + frame_image_name)
    img_copy = copy.deepcopy(img)

    img_copy = draw_point(frame_path, img_copy, frame_txt_name)
    cv2.namedWindow('frame')
    cv2.imshow('frame', img_copy)
    Key = cv2.waitKey()
    if Key == ord('m'):
       img_copy = copy.deepcopy(img)
       modify_data_annotation(frame_path, frame_txt_name)
    if Key == ord('q'):
        sys.exit()
    cv2.destroyAllWindows()

    # w, h, c = imgm.shape
    # print frame_name
    # # img = cv2.resize(img, (1280, 480))
    # img_copy = copy.deepcopy(img)
    # frame_name = frame_name.split('.')[0]
    # cv2.namedWindow('frame')
    # cv2.setMouseCallback('frame', on_EVENT_LBUTTONDOWN)
    # cv2.imshow('frame', img_copy)
    # key = cv2.waitKey()
    # if key == ord('s'):
    #     if len(landmark) == 6:
    #         landmarks = sort_landmark(landmark)
    #         landmarks = str(landmarks)
    #         with open(frame_path + '/' + frame_name + '.txt', 'w') as f:
    #             f.write(landmarks)
    #             f.close()
    #     landmark = []
    #     image_buffer.append(frame_name + '.jpg')
    #     if len(image_buffer) > 10:
    #         image_buffer.pop(0)
    #     flag = 'next'
    #     return flag
    # if key == ord('n'):
    #     flag = 'next'
    #     return flag
    # if key == ord('f'):
    #     flag = 'forward'
    #     image_temporary.append(frame_name + '.jpg')
    #     return flag
    # if key == ord('q'):
    #     sys.exit()
    # cv2.destroyAllWindows()
    # flag = 'next'
    # return flag

def verify_datset_annotation(frame_list, frame_path):
    while(len(frame_list) > 0):
        frame = frame_list.pop()
        verify_data_annotation(frame_path, frame)


def main():
    assert(len(sys.argv)==2)
    frame_path = str(sys.argv[1])
    frame_txt = generate_framelist(frame_path)
    verify_datset_annotation(frame_txt, frame_path)
if __name__ == '__main__':
    main()
