#include "OmniChassis.hpp"
//构造期仅绑定电机指针
OmniChassis(Motor *m1, Motor *m2, Motor *m3, Motor *m4):m1(m1),m2(m2),m3(m3),m4(m4){}

void OmniChassis::Init(){
    //随手上锁好习惯
    IrqGuard guard;
    //首次检查能否正确使能，后续loop检查电机是否真的在线
    Motor_Statue+=m1->enable()+m2->enable()+m3->enable()+m4->enable();
    Motor_Statue_Stack[0]=m1->enable();
    Motor_Statue_Stack[1]=m2->enable();
    Motor_Statue_Stack[2]=m3->enable();
    Motor_Statue_Stack[3]=m4->enable();
    if(m1->feedback().valid)m1->setTorque(0);
    if(m2->feedback().valid)m2->setTorque(0);
    if(m3->feedback().valid)m3->setTorque(0);
    if(m4->feedback().valid)m4->setTorque(0);
    if(Motor_Statue>=3)built=true;
    else built=false;
}

void OmniChassis::Forward_Kinematics(){

}

void OmniChassis::Tick(){
    uint32_t now=HAL_GetTick();
}
