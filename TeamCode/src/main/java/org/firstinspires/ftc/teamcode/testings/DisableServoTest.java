package org.firstinspires.ftc.teamcode.testings;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.lang.reflect.Method;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Disable Servo Test")
@Config
@Disabled
public class DisableServoTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public String servo_name1 = "intakeLeft";
    private Servo servo0 = null;
    public String servo_name2 = "intakeRight";
    private Servo servo2 = null;
    public Method changeServoPWMStatus = null;
    @Override
    public void runOpMode() throws InterruptedException {
        servo0 = hardwareMap.get(Servo.class, servo_name1);
        servo2 = hardwareMap.get(Servo.class, servo_name2);
        ServoController controller = servo0.getController();
        servo0.setPosition(1);
        servo2.setPosition(1);
        int servoPort = servo0.getPortNumber();
        int servoPort2 = servo2.getPortNumber();


        try {
            // 获取方法并保存到变量
            changeServoPWMStatus = LynxServoController.class.getDeclaredMethod("internalSetPwmEnable", int.class, boolean.class);
            changeServoPWMStatus.setAccessible(true); // 绕过权限限制
        } catch (NoSuchMethodException e) {
            telemetry_M.addData("Error", e.getMessage());
            telemetry_M.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry_M.addData("getPortNumber",servoPort);
            telemetry_M.addData("pwm",controller.getPwmStatus());
            telemetry_M.update();
            if (gamepad1.a) {
                try {
                    // 使用反射动态调用方法
                    changeServoPWMStatus.invoke(controller, servoPort, false);
                    changeServoPWMStatus.invoke(controller, servoPort2, false);
                } catch (Exception e) {
                    telemetry_M.addData("Error", e.getMessage());
                    telemetry_M.update();
                }
            } else if (gamepad1.b) {
                try {
                    // 使用反射动态调用方法
                    changeServoPWMStatus.invoke(controller, servoPort, true);
                    changeServoPWMStatus.invoke(controller, servoPort2, true);
                } catch (Exception e) {
                    telemetry_M.addData("Error", e.getMessage());
                    telemetry_M.update();
                }
            }
        }
    }
}