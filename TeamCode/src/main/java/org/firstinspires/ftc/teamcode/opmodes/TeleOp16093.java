package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp(name = "16093TeleOp")
public class TeleOp16093 extends LinearOpMode {
    public SuperStructure upper;
    public NewMecanumDrive drive;
    private Sequences sequence;
    private Pose2d current_pos;
    private Runnable update;
    public int mode=0;//when the sequence is changed, this integer turns to 1 to elicit further control
    public int armPosition;//the desired position of arm
    public int slidePosition;//the desired position of slide
    public double wristPosition;//the desired position of grab servo

    @Override
    public void runOpMode() throws InterruptedException{
        SuperStructure upper = new SuperStructure(
                this,
                () -> {
                    logic_period();
                    drive_period();
                });
        drive = new NewMecanumDrive();
        drive.setUp(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.update();

//        update = ()
//        upper.setUpdateRunnable(update);
        ///////////////////////////GAMEPAD1//////////////////////////////////////////////////////
        XCYBoolean intakeFar =new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean intakeNear = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean resetPos = new XCYBoolean(()->gamepad1.left_stick_button);
        XCYBoolean releaseHigh = new XCYBoolean(()->gamepad1.y);
        XCYBoolean intakeIn = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean intakeOut = new XCYBoolean(()->gamepad1.left_bumper);
        XCYBoolean grabOpen = new XCYBoolean(()->gamepad1.a);
        XCYBoolean grabClose = new XCYBoolean(()->gamepad1.b);
        XCYBoolean wristIntake = new XCYBoolean(()->gamepad1.dpad_left);
        XCYBoolean wristDrop = new XCYBoolean(()->gamepad1.dpad_right);
        XCYBoolean initPos = new XCYBoolean(()->gamepad1.start);
        XCYBoolean armUpSimple = new XCYBoolean(()->gamepad1.back);
        XCYBoolean releaseSpecimen = new XCYBoolean(()->gamepad1.right_trigger>0&&gamepad1.left_trigger>0);

        XCYBoolean resetArm = new XCYBoolean(()-> upper.getTouchSensorPressed());


        ///////////////////////////GAMEPAD2//////////////////////////////////////////////////////

        ///////////////////////////INIT/////////////////////////////////////////////////////////

        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPosition(SSValues.WRIST_DEFAULT);
        //upper.setSlidePosition(SSValues.SLIDE_MIN);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);
        upper.setArmByP(SSValues.ARM_DEFAULT, 0.5);

        sequence = Sequences.RUN;
        waitForStart();
        upper.setIntake(0.5);
        logic_period();

        while(opModeIsActive()) {


            if (intakeFar.toTrue()) {
                //upper.setWristPosition(SSValues.WRIST_DEFAULT);
                sequence = Sequences.INTAKE_FAR;
                mode=1;
                //upper.sleep(1000);
               //upper.setSlidesByP(SSValues.SLIDE_MAX, 0.9);
                //upper.setSlidePosition(SSValues.SLIDE_MAX);
                //upper.sleep(1000);
                //upper.setWristPosition(SSValues.WRIST_INTAKE_FAR);
            }
            if (sequence==Sequences.INTAKE_FAR){
                upper.setArmByP(SSValues.ARM_INTAKE_FAR, 0.5);
                armPosition=SSValues.ARM_INTAKE_FAR;
                slidePosition=SSValues.SLIDE_MAX;
                wristPosition=SSValues.WRIST_INTAKE_FAR;
                upper.setGrabPos(SSValues.GRAB_DEFAULT);
            }
            if (intakeNear.toTrue()) {
                sequence = Sequences.INTAKE_NEAR;
                mode=1;
                //upper.setArmByP(SSValues.ARM_INTAKE_NEAR, 0.5);
                //upper.sleep(300);
                //upper.setSlidesByP(SSValues.SLIDE_MIDDLE, 0.5);
                //upper.sleep(300);
                //upper.setWristPosition(SSValues.WRIST_INTAKE_NEAR);
            }
            if(sequence==Sequences.INTAKE_NEAR){
                upper.setArmByP(SSValues.ARM_INTAKE_NEAR, 0.5);
                armPosition=SSValues.ARM_INTAKE_NEAR;
                slidePosition=SSValues.SLIDE_MIDDLE;
                wristPosition=SSValues.WRIST_INTAKE_NEAR;
                upper.setGrabPos(SSValues.GRAB_DEFAULT);
            }
            if (resetPos.toTrue()) {
                //upper.setGrabPos(SSValues.GRAB_CLOSED);
                //upper.setWristPosition(SSValues.WRIST_INTAKE_NEAR);
                //upper.sleep(500);
                //upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);
                //upper.sleep(700);
                //upper.setSlidePosition(SSValues.SLIDE_MIN);
                //upper.setArmByP(SSValues.ARM_DEFAULT, 0.7);
                //upper.sleep(700);
                //upper.setWristPosition(SSValues.WRIST_DEFAULT);
                //upper.sleep(700);
                sequence = Sequences.RUN;
                mode=1;
            }
            if(sequence==Sequences.RUN){
                upper.setArmByP(SSValues.ARM_DEFAULT, 0.7);
                armPosition=SSValues.ARM_DEFAULT;
                slidePosition=SSValues.SLIDE_MIN;
                wristPosition=SSValues.WRIST_DEFAULT;
                upper.setGrabPos(SSValues.GRAB_CLOSED);
            }
            if (releaseHigh.toTrue()) {
                sequence = Sequences.HIGH_CHAMBER;
                mode=1;
                //upper.setArmByP(SSValues.ARM_UP,0.6);
                //sleep(1500);
                //pper.setSlidePosition(SSValues.SLIDE_MAX);
                //upper.setSlidesByP(SSValues.SLIDE_MAX, 0.9);
                //upper.sleep(1500);
                //upper.setWristPosition(SSValues.WRIST_RELEASE);
            }
            if(sequence==Sequences.HIGH_CHAMBER){
                upper.setArmByP(SSValues.ARM_UP,0.6);
                armPosition=SSValues.ARM_UP;
                slidePosition=SSValues.SLIDE_MAX;
                wristPosition=SSValues.WRIST_RELEASE;
                if (releaseSpecimen.toTrue()){
                    upper.setGrabPos(SSValues.GRAB_OPEN);
                }
            }

            if(armUpSimple.toTrue()){
                upper.setArmPosition(SSValues.ARM_UP);
//                upper.setArmByP(SSValues.ARM_UP,0.5);
            }

            if (gamepad1.right_bumper) {
                upper.setIntake(SSValues.CONTINUOUS_SPIN);
                upper.setIntake(SSValues.CONTINUOUS_SPIN);
            } else if (gamepad1.left_bumper) {
                upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
                upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
            } else {
                upper.setIntake(SSValues.CONTINUOUS_STOP);
                upper.setIntake(SSValues.CONTINUOUS_STOP);
            }

            if(initPos.toTrue()){
                upper.setGrabPos(SSValues.GRAB_DEFAULT);
            }

            if (grabOpen.toTrue()) {
                upper.setGrabPos(SSValues.GRAB_OPEN);
            }
            if (grabClose.toTrue()) {
                upper.setGrabPos(SSValues.GRAB_CLOSED);
            }
            if (wristDrop.toTrue()) {
                upper.setWristPosition(SSValues.WRIST_RELEASE);
            }
            if (wristIntake.toTrue()) {
                upper.setWristPosition(SSValues.WRIST_INTAKE_NEAR);
            }

            if(resetArm.toTrue()){
                upper.resetArmEncoder();
            }
            if (mode==1 &&armPosition-upper.getArmPosition()<300&&armPosition-upper.getArmPosition()>-300){
                mode = 2;
                upper.setSlidesByP(slidePosition,0.6);
            }
            if (mode==2&&slidePosition-upper.getSlidePosition()<300&&armPosition-upper.getSlidePosition()>-300){
                mode = 0;
                upper.setWristPosition(wristPosition);
            }

            drive_period();

            //upper.update();
            telemetry.addData("arm: ", upper.getArmPosition());
            telemetry.addData("slideL: ", upper.getSlideLeftPosition());
            telemetry.addData("slideR: ", upper.getSlideRightPosition());
            telemetry.addData("Arm Error",upper.getArmPosition() - upper.getArmTargetPosition());
            telemetry.addData("Arm Power",upper.getArmPower());
//            telemetry.addData("Front Left: ", drive.getMotorVelo(1));
//            telemetry.addData("Front Back: ", drive.getMotorVelo(2));
//            telemetry.addData("Front Right: ", drive.getMotorVelo(3));
//            telemetry.addData("Back Right: ", drive.getMotorVelo(4));
            telemetry.addData("SlideL Error",upper.getSlideLeftPosition() - upper.getSlideTargetPosition());
            telemetry.addData("SlideR Error",upper.getSlideRightPosition() - upper.getSlideTargetPosition());
            telemetry.addData("Mode",mode);
            telemetry.addData("Arm Diff",armPosition-upper.getArmPosition());
            telemetry.addData("Slide Diff",slidePosition-upper.getSlidePosition());

            telemetry.update();
            XCYBoolean.bulkRead();

        }

    }

    ///////////////////////////OUTSIDE THE LOOP//////////////////////////////////////////////////


    public static enum Sequences {
        RUN,
        INTAKE_FAR,
        INTAKE_NEAR,
        GET_HP,
        HIGH_CHAMBER,
        HIGH_NET
    }

// All of this is very bad and very experimental and also untested
    private void drive_period() {
        drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x, sequence);
//        if (sequence == Sequences.NEAR_INTAKE || sequence == Sequences.FAR_INTAKE) {
//            double x = -gamepad1.left_stick_y * 0.35 + -gamepad1.right_stick_y * 0.65;
//            double y = -gamepad1.left_stick_x * 0.35 + -gamepad1.right_stick_x * 0.65;
//            double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
//            Pose2d power = (new Pose2d(x, y, turn_val * 1)).times(1); //global drive turn ration times global drive power
//            drive.setGlobalPower(power, 1,1); // x_static_compensation, y_static_compensation
//        } else {
//            double x = -gamepad1.left_stick_y * 0.35 + -gamepad1.right_stick_y * 0.65;
//            double y = -gamepad1.left_stick_x * 0.35 + -gamepad1.right_stick_x * 0.65;
//            double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
//            Vector2d fast_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
//            double corrected_rad = fast_stick.angle() - current_pos.getHeading();
//            while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
//            while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
////            if (Math.abs(corrected_rad) < Math.PI / 5) {
////                double div = clamp(
////                        Math.toDegrees(corrected_rad) / 20, 1)
////                        * max_turn_assist_power * fast_stick.norm();
////                turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
////            }
//            Pose2d power = (new Pose2d(x, y, turn_val * 1)).times(1);
//            drive.setGlobalPower(power, 1, 1);
//        }
        drive.update();
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        current_pos = drive.getPoseEstimate();
//        period_time_sec = time.seconds() - last_time_sec;
//        telemetry.addData("elapse time", period_time_sec * 1000);
//        last_time_sec = time.seconds();
        telemetry.update();
    }
}
