#include <opencv2/opencv.hpp>
#include <iostream>
#include <wiringPi.h>

using namespace cv;
using namespace std;

// 定义引脚（根据实际硬件连接修改）
#define LIMIT_SWITCH_PIN 7  // 侧边触碰开关连接的GPIO

// 状态枚举
enum CarStatus { MOVING, AT_STATION, FINISHED };

class StationDetector {
public:
    StationDetector() {
        // 初始化硬件引脚
        wiringPiSetup();
        pinMode(LIMIT_SWITCH_PIN, INPUT);
        pullUpDnControl(LIMIT_SWITCH_PIN, PUD_UP); // 上拉电阻
    }

    // 1. 物理识别：检查侧边触碰开关
    bool checkTouchSwitch() {
        return digitalRead(LIMIT_SWITCH_PIN) == LOW; // 假设低电平触发
    }

    // 2. 视觉识别：检查地面颜色块（例如识别绿色作为分拣点）
    bool checkVisualMarker(Mat& frame) {
        Mat hsv, mask;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // 设置颜色识别范围（以绿色为例）
        Scalar lower_green(35, 43, 46);
        Scalar upper_green(77, 255, 255);
        
        inRange(hsv, lower_green, upper_green, mask);

        // 计算识别到的像素面积
        double area = countNonZero(mask);
        return area > 5000; // 阈值根据摄像头高度调整
    }
};

int main() {
    VideoCapture cap(0); // 开启摄像头
    if (!cap.isOpened()) return -1;

    StationDetector detector;
    CarStatus currentStatus = MOVING;

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // 逻辑判断
        bool isAtStation = detector.checkTouchSwitch() || detector.checkVisualMarker(frame);

        if (isAtStation && currentStatus == MOVING) {
            cout << ">>> 站点识别成功：正在停靠..." << endl;
            // 调用停止电机的函数
            // stopMotors(); 
            currentStatus = AT_STATION;
        }

        // 显示监控（调试用）
        imshow("Station Monitor", frame);
        if (waitKey(30) == 27) break; // 按ESC退出
    }

    return 0;
}