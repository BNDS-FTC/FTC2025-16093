package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.*;







import java.util.ArrayList;


@TeleOp(name = "16093 TeleOp")
public class TeleOp16093 extends LinearOpMode {
    TeleOpDrive drive;
    SuperStructure upper;
    Pose2d current_pos;
    //Runnable update;

    // Modes for system control
    int driveMode = 0; // 0: POV mode; 1: Field-centric mode


    ArrayList<Action> actions = new ArrayList<>(6); // Queue of actions for multi-step operations
    double intakePosition = SSValues.CONTINUOUS_STOP; // Intake servo initial position

//    private final Telemetry telqemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize SuperStructure with periodic functions for logic and drive control
        upper = new SuperStructure(
                this,
                () -> {});
        upper.setUpdateRunnable(() -> {logic_period();drive_period();upper.update();});

        // Initialize and set up mecanum drive, starting position at (0,0,0)
        drive = new TeleOpDrive();
        drive.setUp(hardwareMap);
        //  =====button assignments=====
        // Gamepad 1 button assignments

        XCYBoolean resetPos = new XCYBoolean(() -> gamepad1.left_stick_button);
        XCYBoolean resetOdo = new XCYBoolean(() -> gamepad1.a);
        XCYBoolean switchDrive = new XCYBoolean(() -> gamepad1.back);
        XCYBoolean releaseSample = new XCYBoolean(() -> gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0);

        // Gamepad 2 button assignments

        XCYBoolean intakeFar = new XCYBoolean(() -> gamepad2.dpad_up);
        XCYBoolean intakeNear = new XCYBoolean(() -> gamepad2.dpad_down);
        XCYBoolean releaseHigh = new XCYBoolean(() -> gamepad2.y);
        XCYBoolean releaseLow = new XCYBoolean(() -> gamepad2.a);
        XCYBoolean highChamberAim = new XCYBoolean(() -> gamepad2.right_bumper);
        XCYBoolean liftSlidesSlightly = new XCYBoolean(() -> gamepad2.left_bumper);
        XCYBoolean changeClaw = new XCYBoolean(() -> gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0);

        // =====Initial setup for upper mechanisms to default positions=====


        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);
        upper.setArmByP(SSValues.ARM_DEFAULT, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);

        // Wait until play button is pressed

        waitForStart();


        // Set intake to default stop position and initialize operation mode
        upper.setIntake(SSValues.CONTINUOUS_STOP);

        // Main control loop while op mode is active
        while (opModeIsActive()) {

            /////////////////////////////// OPERATIONS HANDLING ////////////////////////////////////

            // Accepts inputs only if mode is 0 (awaiting input)
            if (actions.isEmpty()) {

                // Resets the position sequence if triggered by resetPos
                if (resetPos.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.RUN);
                    // Sequence actions based on last upper.getSequence()
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.HANG || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                        actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT, 300));
                    }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
                        actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT,200));
                    }
                }

                // High basket release sequences
                if (releaseHigh.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);

                    // Sequence actions for specific release sequencess
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        actions.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 300));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actions.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }

                // Intake sequences and similar conditional checks...
                if (intakeFar.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN ){
                        actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR){
                        actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                    }else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                        actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    }
                }
                if (intakeNear.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                        actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                        actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    }
                }

                if (releaseLow.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.LOW_BASKET);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        actions.add(new ArmAction(upper, SSValues.ARM_LOW_BASKET));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET) {
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE || upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        actions.add(new ArmAction(upper, SSValues.ARM_LOW_BASKET));
                        actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }

                if(liftSlidesSlightly.toTrue() && upper.getSequence() == SuperStructure.Sequences.LOW_BASKET){
                    actions.add(new ArmAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
                }

                //To place the specimen on the chamber, driver 2 presses the right bumper continuously until it can be released.
                if (highChamberAim.toTrue()){
                    upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
                    actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
                    actions.add(new ArmAction(upper, SSValues.ARM_LOW_BASKET));
                    actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM));
                }
                if(highChamberAim.toFalse()){
                    actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE));
                    actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_OPEN, SSValues.CLAW_RIGHT_OPEN));
                }

                //This part allows driver 2 to manually adjust the slide length by power if the upper.getSequence() is intake.
                if((Math.abs(gamepad2.left_stick_y) > -0.1) && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR)){
                    if(gamepad2.left_stick_y > 0 && upper.getSlidesPosition() > 50){
                        upper.setSlidesByPower(-gamepad2.left_stick_y*0.3);
                    }else if(gamepad2.left_stick_y < 0.1 && upper.getSlidesPosition() < SSValues.SLIDE_INTAKE_FAR+50){
                        upper.setSlidesByPower(-gamepad2.left_stick_y*0.3);
                    }else{
                        upper.setSlidesByPower(0);
                    }
                }

                //This part turns off the power of the arm so that it stays in place better after the position is within acceptable range.
                if(Math.abs(upper.getArmPosition()-upper.getArmTargetPosition()) < 30){
                    upper.setArmByP(upper.getArmTargetPosition(), 0);
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

                //Sample released when the arm is in the right place.
                if (releaseSample.toTrue()){
                    upper.setGrabPos(SSValues.GRAB_OPEN);
                }

                //Claw opens/closes when driver 2 presses both triggers.
                if(changeClaw.toTrue()){
                    if(upper.getClawLeft() == SSValues.CLAW_LEFT_OPEN){
                        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
                        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);
                    }
                    else{
                        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
                        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
                    }
                }

                drive_period();
                logic_period();
                upper.update();


            }

            /////////////////////////// DRIVE AND TELEMETRY UPDATES ///////////////////////////

            upper.buildSequence(actions);
//            drive_period();
//            logic_period();
//            upper.update();

        }

    }

    /////////////////////////// SUPPORT METHODS ////////////////////////////

    // Drive control handling for mecanum drive based on selected mode

    private void drive_period() {
        if(upper!= null){
            if (driveMode == 0) {
                drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, upper.getSequence());
            } else {
                drive.setHeadingPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, upper.getSequence());
            }
            drive.updateOdo();
        }
    }

    // Logic updates with telemetry
    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.addData("arm: ", upper.getArmPosition());
        telemetry.addData("slides: ", upper.getSlidesPosition());
        telemetry.addData("Left Slide Velocity", upper.getSlideVelocity());
        telemetry.addData("Left Slide Power:", upper.getSlidePower());
        telemetry.addData("Arm Power",upper.getArmPower());
        telemetry.addData("Current Sequence", upper.getSequence());
        telemetry.addData("Previous Sequence", upper.getPreviousSequence());
        telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("Intake Mode", intakePosition);
        telemetry.addData("Pinpoint Heading: ", drive.getHeading());
//            telemetry_M.addData("Arm Power", upper.getArmPower());
//            telemetry_M.update();

        telemetry.update();
    }
}