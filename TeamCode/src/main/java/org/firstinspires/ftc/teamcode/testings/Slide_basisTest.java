package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@Disabled
@TeleOp(group = "Testing", name = "Slide Test With Power")
public class Slide_basisTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        DcMotorEx mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");

        waitForStart();

        while (!isStopRequested()){
            if(gamepad1.a){
                mSlideRight.setPower(1);
                mSlideLeft.setPower(1);
            }else if(gamepad1.b){
                mSlideRight.setPower(0);
                mSlideLeft.setPower(0);
            }else if(gamepad1.x){
                mSlideRight.setPower(-1);
                mSlideLeft.setPower(-1);
            }
        }
    }
}
