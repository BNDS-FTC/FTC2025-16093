package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.ArmAction;
import org.firstinspires.ftc.teamcode.uppersystems.SlideAction;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.uppersystems.Action;
import org.firstinspires.ftc.teamcode.uppersystems.WristAction;

import java.util.ArrayList;

@TeleOp(name = "16093 TeleOp")
public class TeleOp16093 extends LinearOpMode {
    SuperStructure upper;
    NewMecanumDrive drive;
    Sequences sequence;
    Sequences previousSequence;
    Pose2d current_pos;
    Runnable update;
    int mode=0;//O: when the system is accepting new gamepad inputs. 1: when an input has been passed & a sequence is running.
    int driveMode = 0;//0: POV. 1: Field-centric
    ArrayList<Action> actionSequence = new ArrayList<>();
    double intakePosition = SSValues.CONTINUOUS_STOP;//position of the intake Servo
    boolean resetBoolean = false;

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

        //////////////////.../////////GAMEPAD1//////////////////////////////////////////////////////
        XCYBoolean resetPos = new XCYBoolean(()->gamepad1.left_stick_button);
        XCYBoolean resetOdo = new XCYBoolean(()->gamepad1.a);
        XCYBoolean switchDrive = new XCYBoolean(()->gamepad1.back);
        XCYBoolean releaseSpecimen = new XCYBoolean(()->gamepad1.right_trigger>0&&gamepad1.left_trigger>0);
        XCYBoolean forceStop = new XCYBoolean(()->gamepad1.start);

        XCYBoolean resetArm = new XCYBoolean(()-> upper.getTouchSensorPressed());


        //////////////////////////////GAMEPAD2//////////////////////////////////////////////////////
        XCYBoolean intakeFar =new XCYBoolean(()->gamepad2.dpad_up);
        XCYBoolean intakeNear = new XCYBoolean(()->gamepad2.dpad_down);
        XCYBoolean releaseHigh = new XCYBoolean(()->gamepad2.y);
        XCYBoolean releaseLow = new XCYBoolean(()->gamepad2.a);
//        XCYBoolean customIntakeIncrease = new XCYBoolean(() -> gamepad2.b);
//        XCYBoolean customIntakeDecrease = new XCYBoolean(() -> gamepad2.x);
        XCYBoolean highChamberAim = new XCYBoolean(()->gamepad2.right_bumper);
        XCYBoolean getFromHP = new XCYBoolean(()->gamepad2.left_bumper);
//        XCYBoolean highChamberRelease = new XCYBoolean(()->gamepad2.right_trigger>0);
        XCYBoolean l1Hang = new XCYBoolean(()->gamepad2.back);

        //////////////////////////////INIT//////////////////////////////////////////////////////////

        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);
        upper.setArmByP(SSValues.ARM_DEFAULT, 0.5);

        sequence = Sequences.RUN;
        previousSequence = Sequences.RUN;

        waitForStart();
        //////////////////////////////ON START//////////////////////////////////////////////////////

        upper.setIntake(SSValues.CONTINUOUS_STOP);
        logic_period();
        mode = 0;

        while(opModeIsActive()) {

            ///////////////////////////////BUTTONS//////////////////////////////////////////////////
            if (mode == 0) {
                if (resetPos.toTrue()) {
                    mode = 1;
                    switchSequence(Sequences.RUN);
                    if (previousSequence == Sequences.INTAKE_FAR || previousSequence == Sequences.INTAKE_NEAR || previousSequence == Sequences.CUSTOM_INTAKE) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT,300));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                    } else if (previousSequence == Sequences.HIGH_BASKET || previousSequence == Sequences.HANG || previousSequence == Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE, 100));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT,200));
                    }else if( previousSequence == Sequences.GET_FROM_HP){
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                    }else if (previousSequence == Sequences.HIGH_CHAMBER) {
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_CHAMBER_PLACE, 70));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new WristAction(upper, SSValues.GRAB_OPEN));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                        actionSequence.add(new WristAction(upper, SSValues.GRAB_DEFAULT));
                    }
                }
                if (releaseHigh.toTrue()) {
                    mode = 1;
                    switchSequence(Sequences.HIGH_BASKET);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    if (previousSequence == Sequences.RUN) {
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (previousSequence == Sequences.LOW_BASKET) {
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE, 300));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (previousSequence == Sequences.INTAKE_FAR || previousSequence == Sequences.INTAKE_NEAR || previousSequence == Sequences.CUSTOM_INTAKE) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }
                if (intakeFar.toTrue()) {
                    mode = 1;
                    switchSequence(Sequences.INTAKE_FAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if (previousSequence == Sequences.RUN ){
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR, 200));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    } else if (previousSequence == Sequences.INTAKE_NEAR || previousSequence == Sequences.INTAKE_FAR){
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                    }else if (previousSequence == Sequences.HIGH_BASKET || previousSequence == Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    }
                }
                if (intakeNear.toTrue()) {
                    mode = 1;
                    switchSequence(Sequences.INTAKE_NEAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if (previousSequence == Sequences.RUN) {
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    } else if (previousSequence == Sequences.INTAKE_FAR || previousSequence == Sequences.INTAKE_NEAR) {
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                    } else if (previousSequence == Sequences.HIGH_BASKET || previousSequence == Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    }
                }

                if (releaseLow.toTrue()) {
                    mode = 1;
                    switchSequence(Sequences.LOW_BASKET);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    if (previousSequence == Sequences.RUN) {
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_LOW_BASKET));
                    } else if (previousSequence == Sequences.HIGH_BASKET) {
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (previousSequence == Sequences.INTAKE_FAR || previousSequence == Sequences.INTAKE_NEAR || previousSequence == Sequences.CUSTOM_INTAKE) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actionSequence.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actionSequence.add(new ArmAction(upper, SSValues.ARM_LOW_BASKET));
                        actionSequence.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }

                //These two need to be updated to reflect changes in structure.
                if (highChamberAim.toTrue() && sequence == Sequences.RUN) {
                    mode = 1;
                    switchSequence(Sequences.HIGH_CHAMBER);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    actionSequence.add(new ArmAction(upper, SSValues.ARM_HIGH_CHAMBER_AIM, 70));
                    upper.setWristPos(SSValues.WRIST_HIGH_CHAMBER);
                }
                if (getFromHP.toTrue() && sequence == Sequences.RUN) {
                    mode = 1;
                    switchSequence(Sequences.GET_FROM_HP);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    actionSequence.add(new ArmAction(upper, SSValues.ARM_GET_FROM_HP, 70));
                    upper.setWristPos(SSValues.WRIST_HIGH_CHAMBER);
                }


                //This part allows driver 2 to manually adjust the slide length by power if the sequence is intake.
                if((Math.abs(gamepad2.left_stick_y) > 0) && (sequence == Sequences.INTAKE_NEAR || sequence == Sequences.INTAKE_FAR)){
                    upper.setSlidesToRunByPower();
                    if(upper.getSlidePosition() < 1550 && upper.getSlidePosition() > 50 && (Math.abs(gamepad2.left_stick_y) > 0)){
                        upper.setSlidesByPower(-gamepad2.left_stick_y*0.3);
                    }else{
                        upper.setSlidesByPower(0);
                    }
                }

                //This part turns off the power of the arm so that it stays in place better after the position is within acceptable range.
                if(Math.abs(upper.getArmPosition()-upper.getArmTargetPosition()) < 30){
                    upper.setArmByP(upper.getArmTargetPosition(), 0);
                }

                if(sequence == Sequences.RUN && l1Hang.toTrue()){
                    mode = 1;
                    switchSequence(Sequences.HANG);
                    actionSequence.add(new ArmAction(upper, SSValues.ARM_HANG1));
                }

                //Reset heading
                if(resetOdo.toTrue()){
                    drive.resetOdo();
                }
                //Switch between POV and field-centric drives
                if(switchDrive.toTrue()){
                    if(driveMode==1){
                        driveMode = 0;
                    }else{
                        driveMode = 1;
                    }
                }


                //Intake
                if (gamepad1.right_bumper) {
                    intakePosition = SSValues.CONTINUOUS_SPIN;
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                } else if (gamepad1.left_bumper) {
                    intakePosition = SSValues.CONTINUOUS_SPIN_OPPOSITE;
                    upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
                } else {
                    if(intakePosition == SSValues.CONTINUOUS_SPIN_OPPOSITE){
                        upper.setIntake(SSValues.CONTINUOUS_STOP_OPPOSITE);
                    }
                    else {
                        upper.setIntake(SSValues.CONTINUOUS_STOP);
                    }
                }

                //Specimen released when the arm is in the right place.
                if(sequence==Sequences.HIGH_BASKET||sequence == Sequences.LOW_BASKET){
                    if (releaseSpecimen.toTrue()){
                        upper.setGrabPos(SSValues.GRAB_OPEN);
                    }
                }else if(sequence == Sequences.GET_FROM_HP){
                    if (releaseSpecimen.toTrue()){
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                    }
                }

                //The touch sensor sets a boolean to true. The boolean resets the arm motor's encoders in buildSequence.
                if (resetArm.toTrue()) {
                    resetBoolean = true;
                }else{
                    resetBoolean = false;
                }

            }

            //This is supposed to force a sequence to stop if it meets a deadlock.
            //However, it doesn't work right now and I don't know why.
            if(forceStop.toTrue()){
                mode = 0;
                actionSequence.clear();
            }

            ///////////////////////////NOT GAMEPAD-RELATED//////////////////////////////////////////////

            drive_period();

            telemetry.addData("arm: ", upper.getArmPosition());
            telemetry.addData("slideL: ", upper.getSlideLeftPosition());
            telemetry.addData("slideR: ", upper.getSlideRightPosition());
            telemetry.addData("Arm Power",upper.getArmPower());
            telemetry.addData("Mode",mode);
            telemetry.addData("Current Sequence", sequence);
            telemetry.addData("Previous Sequence", previousSequence);
            telemetry.addData("Drive Mode", driveMode);
            telemetry.addData("Intake Mode", intakePosition);
            telemetry.addData("Pinpoint Heading: ", drive.getHeading());

            telemetry.update();
            XCYBoolean.bulkRead();

            if(mode == 1){
                buildSequence(actionSequence, upper);
            }

        }

    }

    ///////////////////////////OUTSIDE THE LOOP//////////////////////////////////////////////////

    //Runs all the Actions added to the sequence. i only increments once the previous sequence has
    //a small enough error.
    public void buildSequence(ArrayList<Action> actionSequence, SuperStructure upper){
        int i = 0;
        while(i < actionSequence.size()&&opModeIsActive()){
            actionSequence.get(i).actuate();

            //The lines in the middle of these two comments are for specific TeleOp functions.
            drive_period();
            if(resetBoolean){
                upper.resetArmEncoder();
            }
            //The parts outside these two comments are key to the function of buildSequence.

            if (actionSequence.get(i).isFinished()) {
                i++;
            }
        }
        actionSequence.clear();
        mode = 0;
    }

    //Stores the previous sequence and switches a new sequence.
    public void switchSequence(Sequences s){
        previousSequence = sequence;
        sequence = s;
    }

    public enum Sequences {
        RUN,
        INTAKE_FAR,
        INTAKE_NEAR,
        HIGH_BASKET,
        HANG,
        CUSTOM_INTAKE,
        LOW_BASKET,
        HIGH_CHAMBER,
        GET_FROM_HP
        //Etc.
    }

// This is just a normal mecanum drive because everything fancy has been commented out
    private void drive_period() {
        if(driveMode == 0){
            drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x, sequence);
        }else{
            drive.setHeadingPower(gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x, sequence);
        }
        drive.updateOdo();
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
