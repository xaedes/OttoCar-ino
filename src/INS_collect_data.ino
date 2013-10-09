#include <Wire.h>

#include <LSM303.h>
#include <L3G.h>

#include <ros.h>
#include <geometry_msgs/Vector3.h>
// #include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

LSM303 compass;
L3G gyro;

ros::NodeHandle  nh;

geometry_msgs::Vector3 accelerometer_msg;
ros::Publisher accelerometer("/accelerometer/raw", &accelerometer_msg);

geometry_msgs::Vector3 magnetometer_msg;
ros::Publisher magnetometer("/magnetometer/raw", &magnetometer_msg);

geometry_msgs::Vector3 gyroscope_msg;
ros::Publisher gyroscope("/gyroscope/raw", &gyroscope_msg);

void sub_callback_LSM303_CTRL_REG2_A( const std_msgs::UInt8& msg){
    compass.writeAccReg(LSM303_CTRL_REG2_A, msg.data);
}

ros::Subscriber<std_msgs::UInt8> sub_LSM303_CTRL_REG2_A("/LSM303_CTRL_REG2_A", sub_callback_LSM303_CTRL_REG2_A);


void init_accel() {
    // High pass filter mode selection: normal mode 10
    // 10 00 1111 b = 0x8F
    // 01 00 1111 b = 0x4F
    // 00 00 0000 b = 0x00
    compass.writeAccReg(LSM303_CTRL_REG2_A, 0x4F);

    /*
    // Enable Accelerometer
    // 0x3F = 0b00111111
    // Normal power mode, all axes enabled, Output data rate 1000hz
    compass.writeAccReg(LSM303_CTRL_REG1_A, 0x3F);
    /*/ /*
    // Enable Accelerometer
    // 0x2F = 0b00101111
    // Normal power mode, all axes enabled, Output data rate 100hz
    compass.writeAccReg(LSM303_CTRL_REG1_A, 0x2F);
    /*/ //*
    // Enable Accelerometer
    // 0x27 = 0b00100111
    // Normal power mode, all axes enabled, Output data rate 50hz
    compass.writeAccReg(LSM303_CTRL_REG1_A, 0x27);
    //*/
}

void init_gyro() {
    if (!gyro.init())
    {
        Serial.println("Failed to autodetect gyro type!");
        while (1);
    }

    gyro.enableDefault();
    gyro.writeReg(L3G_CTRL_REG1, 0x1F); 
}

void setup() {
    // Serial.begin(9600);
    Wire.begin();

    compass.init();
    compass.enableDefault();

    init_gyro();
    init_accel();

    nh.initNode();
    nh.advertise(accelerometer);
    nh.advertise(magnetometer);
    nh.advertise(gyroscope);
    nh.subscribe(sub_LSM303_CTRL_REG2_A);
}

void loop() {
    // for (int i = 0; i < 4; ++i) {

        compass.read();
        gyro.read();

        const int16_t maxint16 = 32767;
        // int16_t maxint16 = 2047;
        const float fs_gyro = 250.0f;               // full-scale of gyro in [degree per second]
        const float degree2rad = 3.14159f / 180.0f; // factor to convert from degree to rad ; in [rad / degree]
        const float gPerLSB = 0.001f;               // accel scale [g/LSB] ; LSB = least significant bit
        // const float g2ms2 = 1.0f;                // factor to convert from g to m/s^2 ; in [(m/s^2) / g] - for debugging to output in g
        const float g2ms2 = 9.81f;                  // factor to convert from g to m/s^2 ; in [(m/s^2) / g]
        const float fs_magneto = 1.3f;              // full-scale of magneto in [gauss]
        const float gs2ts = 1.0000f;                // factor to convert from gauss to tesla ; in [tesla / gauss]
        // const float gs2ts = 0.0001f;             // factor to convert from gauss to tesla ; in [tesla / gauss]

        // accelerometer_msg.x = (compass.a.x * fs_accel / maxint16) * g2ms2;
        // accelerometer_msg.y = (compass.a.y * fs_accel / maxint16) * g2ms2;
        // accelerometer_msg.z = (compass.a.z * fs_accel / maxint16) * g2ms2;
        accelerometer_msg.x = compass.a.x * (gPerLSB * g2ms2);
        accelerometer_msg.y = compass.a.y * (gPerLSB * g2ms2);
        accelerometer_msg.z = compass.a.z * (gPerLSB * g2ms2);

        magnetometer_msg.x = compass.m.x * (fs_magneto / maxint16 * gs2ts);
        magnetometer_msg.y = compass.m.y * (fs_magneto / maxint16 * gs2ts);
        magnetometer_msg.z = compass.m.z * (fs_magneto / maxint16 * gs2ts);

        gyroscope_msg.x = gyro.g.x * (fs_gyro / maxint16 * degree2rad);
        gyroscope_msg.y = gyro.g.y * (fs_gyro / maxint16 * degree2rad);
        gyroscope_msg.z = gyro.g.z * (fs_gyro / maxint16 * degree2rad);
    // }
    accelerometer.publish(&accelerometer_msg);
    magnetometer.publish(&magnetometer_msg);
    gyroscope.publish(&gyroscope_msg);

    nh.spinOnce();
    // delay(10);
}
