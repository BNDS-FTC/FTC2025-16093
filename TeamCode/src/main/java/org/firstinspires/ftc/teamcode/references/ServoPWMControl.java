package org.firstinspires.ftc.teamcode.references;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import java.lang.reflect.Method;


public class ServoPWMControl{
    Servo servo=null;
    ServoController controller = null;
    Method changeServoPWMStatus = null;
    int ServoPort;
    public ServoPWMControl(Servo servo){
        this.servo=servo;
        controller=this.servo.getController();
        ServoPort=servo.getPortNumber();
        try {
            // 获取方法并保存到变量
            changeServoPWMStatus = LynxServoController.class.getDeclaredMethod("internalSetPwmEnable", int.class, boolean.class);
            changeServoPWMStatus.setAccessible(true); // 绕过权限限制
        } catch (NoSuchMethodException e) {
            throw new RuntimeException("TM的反射炸了!");
        }
   }
   public void setStatus(boolean enable){
       try {
           changeServoPWMStatus.invoke(controller, ServoPort, enable);
       } catch (Exception e) {
           throw new RuntimeException("TM的调用方法炸了!");
       }
   }
}
