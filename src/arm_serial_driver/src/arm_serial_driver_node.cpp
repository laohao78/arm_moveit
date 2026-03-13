#include "arm_serial_driver/arm_serial_driver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_serial_driver");
    ros::NodeHandle nh("~"); // 使用私有命名空间加载参数

    arm_serial_driver::ArmSerialDriver driver(nh);
    driver.run();

    return 0;
}