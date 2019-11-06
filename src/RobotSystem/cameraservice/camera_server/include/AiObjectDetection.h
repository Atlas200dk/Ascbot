#ifndef _AIOBJECTDETECTION_H_
#define _AIOBJECTDETECTION_H_

#include <sys/types.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <errno.h>
#include <cmath>
#include "HiRoadObjectFollowing.h"

static bool test_collision(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
    return ( ( (x1 >=x3 && x1 < x4) || (x3 >= x1 && x3 <= x2) ) &&
        ( (y1 >=y3 && y1 < y4) || (y3 >= y1 && y3 <= y2) ) ) ? true : false;
}

static int CheckObstacleResults(std::vector<hi::RoadObjectData> detection_results, int width, int height,
        float scale_x1, float scale_y1, float scale_x2, float scale_y2, float scale_score) {
    int car_safe_window_x1 = width*scale_x1;
    int car_safe_window_y1 = height*scale_y1;
    int car_safe_window_x2 = width*scale_x2;
    int car_safe_window_y2 = height*scale_y2;
    for (vector<hi::RoadObjectData>::const_iterator iter = detection_results.cbegin();
    iter != detection_results.cend(); iter++) {
        hi::RoadObjectData one_result = *iter;
        if (one_result.lable >= 0 && one_result.lable <= 6) {
            float _score = static_cast<float>(one_result.confidence);
            if (_score >= scale_score) {
                if (test_collision(car_safe_window_x1, car_safe_window_y1, car_safe_window_x2, car_safe_window_y2,
                one_result.x, one_result.y, one_result.x+one_result.width, one_result.y+one_result.height))
                return 1;
            }
        }
    }
    return 0;
}

/*
 * detection_results: detection result
 * label_result: label result
 * width: input image width
 * height: input image  height
 * angle: out for angle
 * return: 0
 */
static int checkTableResults(std::vector<hi::RoadObjectData> detection_results, int width, int height, int& angle) {
    for (vector<hi::RoadObjectData>::const_iterator iter = detection_results.cbegin();
    iter != detection_results.cend(); iter++) {
        hi::RoadObjectData one_result = *iter;
        if (one_result.lable <= 7) { // obstacle is object
            int object_xmax = one_result.x + one_result.width;
            int object_xmin = one_result.x;
            int middle_line = width / 2;
            int object_mid = (object_xmax + object_xmin) / 2; 
            angle = -90;    // turn left as default
            if (object_mid >= middle_line) {    //object is in front-right of car
                angle = -90; // turn left
            }
            else {
                angle = 90;  //turn right
            }
            return 0;
        }
    }
    return -1;
}

vector<float> v_angles;  // cache for last angle values
/*
 * detection_results: detection result
 * label_result: label result
 * width: input image width
 * height: input image  height
 * angle: out for angle
 * point_x: out for x value of point
 * point_y: out for y value of point
 * return: 0
 */
static int checkRoadLineResults(std::vector<hi::RoadObjectData> detection_results, int width, int height, int& angle) {
    for (vector<hi::RoadObjectData>::const_iterator iter = detection_results.cbegin();
    iter != detection_results.cend(); iter++) {
        hi::RoadObjectData one_result = *iter;
        if (one_result.lable > 7 && (one_result.lable < 11)) {
           int xmax = one_result.x + one_result.width;
           int xmin = one_result.x;
           int ymax = one_result.y + one_result.height;
           int ymin = one_result.y;

           int _w_ = xmax-xmin;
           int _h_ = ymax-ymin;

           float px = xmin;
           float py = ymin;

           if (one_result.lable == 8) {
               px = (xmin+xmax)*0.5;
               py = (ymin+ymax)*0.5;
           } else if (one_result.lable == 9) {
               px = xmin+_w_*0.5-_w_*0.1;
               py = ymin+_h_*0.5-_h_*0.5;
           } else if (one_result.lable == 10) {
               px = xmin+_w_*0.5-_w_*0.1;
               py = ymin+_h_*0.5-_h_*0.5;
           }

           angle = 0.0;

           if (py < height) {
               if ((abs(px-width/2) > 0.00001)) {
                   angle = 90-atan((height-py)/abs(px-width/2.0))*180/(3.14159);
                   if (px-width/2 < 0) {
                     angle =- angle;
                   }
               }
           }
            const int vector_size = 2;
            v_angles.push_back(angle);
            float weight[] = {0.00, 0.0, 1.0};  // weight for filter the values
            if (v_angles.size() > vector_size) {
                float temp_angle = 0;
                int cnt_angle = 0;
                for (vector<float>::iterator it=v_angles.begin(); it != v_angles.end(); it++) {
                    temp_angle = temp_angle+(*it)*weight[cnt_angle];
                    cnt_angle++;
                }
                angle = temp_angle;
                v_angles.erase(v_angles.begin());
            }
            return 0;
        }
    }
    return -1;
}


static int CheckEdge(std::vector<hi::RoadObjectData> detection_results, int width, int height, float scale_score) { // 0 return left,1 return right
    hi::RoadObjectData min_one_result;
    for (vector<hi::RoadObjectData>::const_iterator iter = detection_results.cbegin();
    iter != detection_results.cend(); iter++) {
        hi::RoadObjectData one_result = *iter;

        if (one_result.lable == 12) {// edge
            float _score = static_cast<float>(one_result.confidence);
            if (_score >= scale_score) {
                if(iter == detection_results.cbegin())
                {
                    min_one_result = *iter;
                    continue;
                }
                float x1 = one_result.x + one_result.width/2 - width/2;
                float y1 = one_result.y + one_result.height - height;
                float x2 = min_one_result.x + min_one_result.width/2 - width/2;
                float y2 = min_one_result.y + min_one_result.height - height;
                if((x1*x1+y1*y1) - (x2*x2+y2*y2) < 0)
                    min_one_result = one_result;
            }
        }
    }
    if(min_one_result.x-width/2 < 0)
        return 0;//left
    if(min_one_result.x-width/2 > 0)
        return 1;//right
}



#endif  // _AIOBJECTDETECTION_H_
