package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.references.XCYBoolean;

@TeleOp
@Config
public class ArmAdjustment extends LinearOpMode{
    public static double armPowerUp = 1;
    public static double armPowerDown = 0.8;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.left_stick_y > 0){
                arm.setPower(gamepad1.left_stick_y*armPowerUp);
            }else if(gamepad1.left_stick_y < 0){
                arm.setPower(gamepad1.left_stick_y*armPowerDown);
            }
            else{
                arm.setPower(0);
            }

        }
    }
}
