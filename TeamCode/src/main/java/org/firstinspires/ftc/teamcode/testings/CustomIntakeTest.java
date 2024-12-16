package org.firstinspires.ftc.teamcode.testings;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(group = "Testing")
@Config
@Disabled
public class CustomIntakeTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int encoder_position = 1150;
    public static int slide_positon = 0;
    public static double max_power = 1;
    public static boolean read_only = true;
    public static boolean reverse_0 = false;
    public static boolean reset = true;
    public static boolean set_power_mode_or_set_position_mode = false;
    public static String motor_name_0 = "arm";
    public static double servo_pos1 = 0.5;
    public static String servo_name1 = "wrist";
    private Servo servo0=null;



    @Override
    public void runOpMode() {
        servo0 = hardwareMap.get(Servo.class, servo_name1);
        DcMotorEx motor0 = hardwareMap.get(DcMotorEx.class, motor_name_0);
        DcMotorEx mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (reset) {
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (reverse_0) {
            motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while (opModeIsActive()) {
            mSlideRight.setTargetPosition(slide_positon);
            motor0.setTargetPosition(encoder_position);
            mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mSlideRight.setPower(0.5);
            motor0.setPower(0.5);

            servo0.setPosition(servo_pos1);
//                servo1.setPosition(servo_pos2);
            telemetry_M.addData("leftFront", servo0.getPosition());
//                telemetry_M.addData("rightfront", servo1.getPosition());
            telemetry_M.update();

//            if (set_power_mode_or_set_position_mode) {
//                if (read_only) {
//                    motor0.setPower(0);
//                }
//                else {
//                    motor0.setPower(max_power);
//                }
//                motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            } else {
//                if (!read_only) {
//                    motor0.setTargetPosition(encoder_position);
//                    motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motor0.setPower(max_power);
//
//                    sleep(10000);
//                    motor0.setTargetPosition(0);
//                    motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motor0.setPower(max_power);
//                }
//                telemetry_M.addData("is busy_1", motor0.isBusy());
////                telemetry_M.addData("encoder_1", motor0.getCurrentPosition());
////                telemetry_M.addData("is busy_3", motor1.isBusy());
////                telemetry_M.addData("encoder_2", motor1.getCurrentPosition());
//            }
            telemetry_M.addData("encoder_0", motor0.getCurrentPosition());
            telemetry_M.addData("velocity_1", motor0.getVelocity());
            telemetry_M.update();
        }
    }
}