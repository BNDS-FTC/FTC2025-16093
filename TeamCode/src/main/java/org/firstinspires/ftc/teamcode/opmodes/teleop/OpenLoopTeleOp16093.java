//package org.firstinspires.ftc.teamcode.opmodes.teleop;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.SuperStructure;
//import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
//import org.firstinspires.ftc.teamcode.references.SSValues;
//import org.firstinspires.ftc.teamcode.references.XCYBoolean;
//
//@Config
//@TeleOp(name = "Open Loop TeleOP")
//public class OpenLoopTeleOp16093 extends LinearOpMode {
//    public double slideOpenloopConst = 1;
//    public double armOpenloopConst = 1;
//    public double ascentPos = 1;
//    @Override
//    public void runOpMode() throws InterruptedException{
//        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
//        SuperStructure upper = new SuperStructure(this,()->{},0);
//
//        XCYBoolean changeAscent = new XCYBoolean(() -> gamepad2.dpad_down);
//        upper.setAscentState(SuperStructure.AscentState.ASCENT_UP);
//
//        DcMotorEx armUp = hardwareMap.get(DcMotorEx.class, "armUp");
//        DcMotorEx armDown = hardwareMap.get(DcMotorEx.class, "armDown");
//
//        armUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        armUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        waitForStart();
//
//
//        while(opModeIsActive()){
//
//            upper.setSlidesByPower(SSValues.SLIDE_MIN, -gamepad2.right_stick_y * slideOpenloopConst);
//
//            if(changeAscent.toTrue()){
//                if(ascentPos==1){
//                    ascentPos = 0.3;
//                } else if (ascentPos==0.3) {
//                    ascentPos = 0;
//                }else
//                {
//                    ascentPos = 1;
//                }
//            }
//
//            if(gamepad2.left_stick_y > 0){
//                telemetry.addData("Arm State", "up");
//                armUp.setPower(-gamepad2.left_stick_y*armOpenloopConst);
//                armDown.setPower(-gamepad2.left_stick_y*armOpenloopConst);
//
//            }else if(gamepad2.left_stick_y < 0){
//                telemetry.addData("Arm State", "down");
//                armUp.setPower(-gamepad2.left_stick_y*armOpenloopConst);
//                armDown.setPower(-gamepad2.left_stick_y*armOpenloopConst);
//            }else{
//                telemetry.addData("Arm State", "stop");
//                armUp.setPower(0);
//                armDown.setPower(0);
//            }
//
//            drive.setFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, null);
//            drive.update();
//
//            telemetry.addData("Arm Position: ", upper.getArmPosition());
//            telemetry.addData("Slide Position: ", upper.getSlidesPosition());
//            telemetry.addLine("");
//            telemetry.addData("Arm Power", upper.getArmPower());
//            telemetry.addData("Slide Power:", upper.getSlidePower());
//
//            telemetry.update();
//
//            upper.update();
//            XCYBoolean.bulkRead();
//        }
//    }
//}