//package org.firstinspires.ftc.teamcode.opmodes.teleop;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.SuperStructure;
//import org.firstinspires.ftc.teamcode.actions.actioncore.Action;
//import org.firstinspires.ftc.teamcode.actions.ArmAction;
//import org.firstinspires.ftc.teamcode.actions.SlideAction;
//import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
//import org.firstinspires.ftc.teamcode.references.SSValues;
//import org.firstinspires.ftc.teamcode.references.XCYBoolean;
//
//@Config
//@TeleOp(name = "Close Loop Ascend")
//public class CloseLoopAscend extends LinearOpMode {
//    public double slideOpenloopConst = 1;
//    public double armOpenloopConst = 1;
//    public double ascentPos = 1;
//    NewMecanumDrive drive;
//    SuperStructure upper;
//    @Override
//    public void runOpMode() throws InterruptedException{
//        Runnable update;
//        drive = new NewMecanumDrive(hardwareMap);
//        upper = new SuperStructure(
//                this,
//                () -> {
//                }, 0);
//        XCYBoolean changeAscent = new XCYBoolean(() -> gamepad1.dpad_down);
////        upper.setAscentPos(ascentPos);
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
//        upper.resetSlide();
//
//        update = () -> {
//            logic_period();
//            drive_period();
//            upper.update();
////            gamepad_inputs();
////            if (forceStop.toTrue()) {
////                Action.stopBuilding = true;
////            }
////            if (forceStop.toFalse()) {
////                Action.stopBuilding = false;
////            }
//        };
//
//        waitForStart();
//
//        while(opModeIsActive()) {
////            upper.setAscentPos(ascentPos);
//
//            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
//            Action.buildSequence(update);
//
////            if(changeAscent.toTrue()){
////                if(ascentPos==1){
////                    ascentPos = 0;
////                }else{
////                    ascentPos = 1;
////                }
////            }
//
////            if(gamepad2.left_stick_y > 0){
////                telemetry.addData("Arm State", "up");
////                armUp.setPower(-gamepad2.left_stick_y*armOpenloopConst);
////                armDown.setPower(-gamepad2.left_stick_y*armOpenloopConst);
////
////            }else if(gamepad2.left_stick_y < 0){
////                telemetry.addData("Arm State", "down");
////                armUp.setPower(-gamepad2.left_stick_y*armOpenloopConst);
////                armDown.setPower(-gamepad2.left_stick_y*armOpenloopConst);
////            }else{
////                telemetry.addData("Arm State", "stop");
////                armUp.setPower(0);
////                armDown.setPower(0);
////            }
//
////            if (gamepad1.a) {
////                upper.setAscentPos(1);
////            }
//            //drive
//            if(gamepad1.b){
//                upper.setAscentPos(0);
//                Action.actions.add(new ArmAction(upper, SSValues.ARM_ASCENT_AIM, 10));
//                Action.buildSequence(update);
//                sleep(1000);
//                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 10));
//                Action.buildSequence(update);
//                sleep(1000);
//                Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 10));
//                Action.buildSequence(update);
//                sleep(1000);
//                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_YELLOW, 10));
//                Action.buildSequence(update);
//                sleep(1000);
//                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 10));
//                Action.buildSequence(update);
//                sleep(1000);
//                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
//                Action.buildSequence(update);
//            }
//
//            telemetry.addData("Arm Position: ", upper.getArmPosition());
//            telemetry.addData("Slide Position: ", upper.getSlidesPosition());
//            telemetry.addLine("");
//            telemetry.addData("Arm Power", upper.getArmPower());
//            telemetry.addData("Slide Power:", upper.getSlidePower());
//            telemetry.update();
//
//            upper.update();
//            XCYBoolean.bulkRead();
//        }
//    }
//    private void drive_period(){
//        drive.setBotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
//    }
//    private void logic_period() {
//        XCYBoolean.bulkRead();
//        telemetry.update();
//    }
//}