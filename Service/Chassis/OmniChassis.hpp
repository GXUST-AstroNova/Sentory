#include "motor.hpp"
#include "pid.hpp"
#inclue "main.h"

struct Vector3D{
    float x;
    float y;
    float z;
}
struct Vector4D{
    float m1;
    float m2;
    float m3;
    float m4;
}

/** 
 * @brief 全向底盘类 
 * @warning 该全向轮类默认四轮全向轮，且四轮电机均为同型号电机，且电机安装角度为45°，
*/
class OmniChassis{
    public:
        OmniChassis();
        //四个全向轮电机指针,控制侧使用力矩接口
        Motor *m1;
        Motor *m2;
        Motor *m3;
        Motor *m4;
        //不要再构造期初始化，在电机等外设完成初始化后，进行底盘等硬件初始化
        void Init();
        void Tick();
        //SI单位制下的底盘力控PID
        //TODO:PID调参
        PositionalPID Linar_x_Force(1.0f, 0.0f, 0.0f, 10.0f, 0.01f, 100.0f);
        PositionalPID Linar_y_Force(1.0f, 0.0f, 0.0f, 10.0f, 0.01f, 100.0f);
        PositionalPID Angular_Force(1.0f, 0.0f, 0.0f, 10.0f, 0.01f, 100.0f);
        //前向动力学计算，返回底盘在x,y方向的线速度和角速度
        Vector3D Forward_Kinematics();
        Vector4D Inverse_Kinematics(float Linar_x_Force, float Linar_y_Force, float Angular_Force);
        //将指令力矩转化为实际电机应输出的力矩
        float CMD_Tor_To_Motor_Tor(float Force){
            return Force * Gear_Ratio * Wheel_Radius;
        }
    private:
        bool built=false;
        uint32_t last_tick_ms=0;
        //当某个电机的返回帧相对于Tick时间差超过该阈值时，认为该电机离线
        uint32_t offline_ms_threshold=500;
        //底盘构建时检测在线轮数，运行时持续维护，用于处理降级情况
        uint8_t Motor_Statue=0;
        bool Motor_Statue_Stack[4]={false,false,false,false};
        //底盘期望速度和观测速度
        Vector3D exp_Velocity;
        Vector3D obs_Velocity;
        float Gear_Ratio= 268.0f / 17.0f;
        float Wheel_Radius= .075f;
        //动力学矩阵
        float matric[4][4]={
            {1.0f, -1.0f, -1.0f, 1.0f},
            {1.0f, 1.0f, 1.0f, 1.0f},
            {-1.0f, -1.0f, 1.0f, 1.0f},
            {Gear_Ratio * Wheel_Radius, Gear_Ratio * Wheel_Radius, Gear_Ratio * Wheel_Radius, Gear_Ratio * Wheel_Radius}
        };

}