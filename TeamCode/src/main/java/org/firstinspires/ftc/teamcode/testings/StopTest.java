//package org.firstinspires.ftc.teamcode.testings;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//
//public class StopTest extends LinearOpMode
//{
//    DcMotorEx motor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        motor = hardwareMap.get(DcMotorEx.class,"motor");
//        waitForStart();
//
//        while(isStopRequested()){
//            motor.setPower(0.5);
//        }
//    }
//}
