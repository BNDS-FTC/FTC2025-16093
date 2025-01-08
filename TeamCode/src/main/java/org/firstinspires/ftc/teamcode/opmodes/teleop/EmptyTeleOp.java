package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.ArmAction;
import org.firstinspires.ftc.teamcode.actions.GrabAction;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.WristAction;

import java.util.List;

@TeleOp(name = "Empty TeleOp")
public class EmptyTeleOp extends LinearOpMode {
    NewMecanumDrive drive;
    SuperStructure upper;
    Pose2d current_pos;
    Runnable update;
    private List<LynxModule> allHubs;
    int count = 0;
    double oldTime = 0;

    // Modes for system control
    int driveMode = 0; // 0: POV mode; 1: Field-centric mode
    public static int slideMode = 0;//1: setpower
    boolean intakeAct = false;
    double slideOpenloopConst = 0.3;
    double intakePosition = SSValues.CONTINUOUS_STOP; // Intake servo initial position
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    XCYBoolean resetPos, resetOdo, changeGrab, slideLonger,slideShorter, forceStop, lockSlide, releaseHigh, releaseLow, switchDrive, autoToggleDriveMode
            ,highChamberAim, liftSlidesSlightly, changeClaw, wristHeightSwitch, armDownByPower, manualResetEncoders, resetArm, goToLastStoredPos, storeThisPos;

    @Override
    public void runOpMode() throws InterruptedException {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        // Initialize SuperStructure with periodic functions for logic and drive control
        upper = new SuperStructure(
                this,
                () -> {
                }, 0);

        //  =====button assignments=====
        // Gamepad 1 button assignments

        resetPos = new XCYBoolean(() -> gamepad1.left_stick_button && !gamepad1.right_stick_button);
        resetOdo = new XCYBoolean(() -> gamepad1.right_stick_button && !gamepad1.left_stick_button);
        switchDrive = new XCYBoolean(() -> gamepad1.right_stick_button && gamepad1.left_stick_button);
        changeGrab = new XCYBoolean(() -> gamepad1.right_trigger > 0);
        slideLonger = new XCYBoolean(() -> gamepad1.dpad_up);
        slideShorter = new XCYBoolean(() -> gamepad1.dpad_down);
        forceStop = new XCYBoolean(() -> gamepad1.b);
        lockSlide = new XCYBoolean(() -> gamepad1.y);
        releaseHigh = new XCYBoolean(() -> gamepad1.y);
        releaseLow = new XCYBoolean(() -> gamepad1.a);
        highChamberAim = new XCYBoolean(() -> gamepad1.right_bumper);
        liftSlidesSlightly = new XCYBoolean(() -> gamepad1.left_bumper);
        changeClaw = new XCYBoolean(() -> gamepad1.left_trigger > 0);
        wristHeightSwitch = new XCYBoolean(() -> gamepad1.right_stick_button);
        armDownByPower = new XCYBoolean(() -> gamepad1.options && !(gamepad1.back));
        manualResetEncoders = new XCYBoolean(() -> gamepad1.back && gamepad1.options);
        goToLastStoredPos = new XCYBoolean(() -> gamepad1.dpad_left);
        storeThisPos = new XCYBoolean(() -> gamepad1.dpad_right);

        resetArm = new XCYBoolean(() -> upper.getTouchSensorPressed());

        drive = new NewMecanumDrive(hardwareMap);

        update = () -> {
            drive_period();
            upper.update();
            gamepad_inputs();
            if (forceStop.toTrue()) {
                Action.stopBuilding = true;
            }
            if (forceStop.toFalse()) {
                Action.stopBuilding = false;
            }

            if (Action.actions.isEmpty() && resetArm.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN && (Math.abs(upper.getSlideError()) < 10 || upper.getSlideMode() == DcMotor.RunMode.RUN_USING_ENCODER)) {
                upper.resetArmEncoder();
                upper.resetSlideEncoder();
            }

            if(autoToggleDriveMode.toTrue()){
                toggleDriveMode(1);
            }
            if(autoToggleDriveMode.toFalse()){
                toggleDriveMode(0);
            }
            logic_period();

            if(drive.simpleMoveIsActivate){
                drive.update();
            }else{
                drive.updatePoseEstimate();
            }


        };

        // Initialize and set up mecanum drive, starting position at (0,0,0)
        drive.setUpdateRunnable(update);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
        drive.update();


        // =====Initial setup for upper mechanisms to default positions=====


        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.1);
        upper.setArmByP(SSValues.ARM_DOWN, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);
        upper.unlockSlide();

        drive.storeCurrentPos();
        drive.resetOdo();
        Action.actions.clear();
        autoToggleDriveMode = new XCYBoolean(() -> upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET && !drive.simpleMoveIsActivate);


        // Wait until play button is pressed

        waitForStart();


        // Set intake to default stop position and initialize operation mode
        upper.setIntake(SSValues.CONTINUOUS_STOP);
//        upper.startIntake();

        // Main control loop while op mode is active
        while (opModeIsActive() && !isStopRequested()) {
            update.run();
            Action.buildSequence(update);

        }
    }


    /////////////////////////// SUPPORT METHODS ////////////////////////////

    // Drive control handling for mecanum drive based on selected mode

    private void gamepad_inputs() {
        if (Action.actions.isEmpty() && resetArm.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN && (Math.abs(upper.getSlideError()) < 10 || upper.getSlideMode() == DcMotor.RunMode.RUN_USING_ENCODER)) {
            upper.resetArmEncoder();
            upper.resetSlideEncoder();
        }

        /////////////////////////////// OPERATIONS HANDLING ////////////////////////////////////

        // Accepts inputs only if mode is 0 (awaiting input)
        if (Action.actions.isEmpty()) {
            // Resets the position sequence if triggered by resetPos
            if (resetPos.toTrue()) {
                driveMode = 0;
                upper.switchSequence(SuperStructure.Sequences.RUN);
                upper.setIntake(SSValues.CONTINUOUS_STOP);
//                    upper.stopIntake();
                // Sequence actions based on last sequence
                if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE || upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {

                    Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED, 60));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 900));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.ASCENT || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER) {
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));

                }
            }

            // High basket release sequences
            if (releaseHigh.toTrue()) {
                upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
                upper.setGrabPos(SSValues.GRAB_CLOSED);
                drive.storeCurrentPos();
                if(!drive.simpleMoveIsActivate){
                    driveMode = 1;
                }
                // Sequence actions for specific release sequences
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 100));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
                    Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED, 60));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                }
            }

            // Intake sequences and similar conditional checks...
            if (slideLonger.toTrue() && upper.getSequence() != SuperStructure.Sequences.LOW_BASKET) {
                toggleDriveMode(0);
                upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
//                    drive.storeCurrentPos();
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                    Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR) {
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET) {
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                }
            }
            if (slideShorter.toTrue() && upper.getSequence() != SuperStructure.Sequences.LOW_BASKET) {
                toggleDriveMode(0);
                upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                    Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET) {
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                }
            }

            if (releaseLow.toTrue()) {
                upper.switchSequence(SuperStructure.Sequences.LOW_BASKET);
                upper.setGrabPos(SSValues.GRAB_CLOSED);
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET) {
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    Action.actions.add(new SlideAction(upper, 0));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                }
                else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE || upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER) {
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                    Action.actions.add(new SlideAction(upper, 0));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                }
            }

            if (liftSlidesSlightly.toTrue() && upper.getSequence() == SuperStructure.Sequences.LOW_BASKET) {
                upper.switchSequence(SuperStructure.Sequences.LOW_BASKET);
                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
            }

            //To place the specimen on the chamber, driver 2 presses the right bumper continuously until it can be released.
            if (highChamberAim.toTrue() && upper.getSequence() == SuperStructure.Sequences.LOW_BASKET) {
                upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
                drive.storeCurrentPos();
                Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_TELEOP));
            }
            if (highChamberAim.toFalse() && upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER) {
                upper.switchSequence(SuperStructure.Sequences.RUN);
                Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
                Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 200));
            }

            //This part allows driver 2 to manually adjust the slide length by power if the upper.getSequence() is intake.
//                if(customSetSlide.toTrue()){
//
//                }
            if ((Math.abs(gamepad1.right_stick_y) > 0.3) && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR)) {
                slideMode = 1;
                if (upper.getWristPosition() == SSValues.WRIST_INTAKE) {
                    slideOpenloopConst = 0.2;
                } else {
                    slideOpenloopConst = 0.6;
                }
                if (gamepad1.right_stick_y > 0.3 && upper.getSlidesPosition() > 100) {
                    upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, -gamepad1.right_stick_y * slideOpenloopConst);
                } else if (gamepad1.right_stick_y < -0.3 && upper.getSlidesPosition() < SSValues.SLIDE_INTAKE_FAR - 100) {
                    upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, -gamepad1.right_stick_y * slideOpenloopConst);
                } else {
                    upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, 0);
                }
            } else if (upper.getSlideMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                upper.setSlidesByPower(upper.getSlideTargetPosition(), 0);
            }

            //Ascending
            if (slideLonger.toTrue() && upper.getSequence() == SuperStructure.Sequences.LOW_BASKET) {
                upper.switchSequence(SuperStructure.Sequences.ASCENT);
                Action.actions.add(new SlideAction(upper, SSValues.SLIDE_ASCENT_UP));
            }
            if (slideShorter.toTrue() && upper.getSequence() == SuperStructure.Sequences.ASCENT) {
                upper.setSlidesByP(SSValues.SLIDE_ASCENT_DOWN, 1);
            }

            //This part allows driver 2 to manually move the arm down.
            if (armDownByPower.toTrue()) {
                upper.switchSequence(SuperStructure.Sequences.RUN);
                upper.setArmByPower(SSValues.ARM_DOWN, -1);
            }
            if (armDownByPower.toFalse()) {
                upper.setArmByPower(SSValues.ARM_DOWN, 0);
            }
//                if(armDownByPower.toFalse()){
//                    upper.resetArmEncoder();
//                }
            if (gamepad1.back && !gamepad1.options) {
                upper.setSlidesByPower(SSValues.SLIDE_MIN, -1);
            }

            if (manualResetEncoders.toTrue()) {
                drive.simpleMoveIsActivate = false;
                driveMode = 0;
                upper.setWristPos(SSValues.WRIST_DEFAULT);
                upper.setArmByPower(-SSValues.ARM_HANG1, -1);
            }
            if (manualResetEncoders.toFalse()) {
                upper.resetArmEncoder();
                upper.resetSlideEncoder();
//                drive.resetOdo();
                upper.setArmByPower(SSValues.ARM_DOWN, 0);
                upper.setSlidesByPower(SSValues.SLIDE_MIN, 0);
            }

            //Reset heading
            if (resetOdo.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN) {
                drive.resetHeading();
            }

            if (gamepad1.right_bumper && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET)) {
                intakePosition = SSValues.CONTINUOUS_SPIN;
                upper.setIntake(SSValues.CONTINUOUS_SPIN);
            } else if (gamepad1.left_bumper && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET)) {
                intakePosition = SSValues.CONTINUOUS_SPIN_OPPOSITE;
                upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
            } else {
                upper.setIntake(SSValues.CONTINUOUS_STOP);
            }

            //Sample released when the arm is in the right place.
            if (changeGrab.toTrue()) {
                if(upper.getSequence() != SuperStructure.Sequences.INTAKE_FAR && upper.getSequence() != SuperStructure.Sequences.INTAKE_NEAR && upper.getSequence() != SuperStructure.Sequences.CUSTOM_INTAKE){
                    if (upper.getGrabPos() != SSValues.GRAB_OPEN) {
                        upper.setGrabPos(SSValues.GRAB_OPEN);
                    } else {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    }
                }else{
                    if (upper.getGrabPos() != SSValues.GRAB_CLOSED) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        upper.setWristPos(SSValues.WRIST_DEFAULT);
                    } else {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                    }
                }
            }

            if ((upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR) && wristHeightSwitch.toTrue()) {
                if (upper.getWristPosition() != SSValues.WRIST_INTAKE) {
                    upper.setWristPos(SSValues.WRIST_INTAKE);
                } else {
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                }
            }

            //Claw opens/closes when driver 1 presses right trigger.
            if (changeClaw.toTrue()) {
                if (upper.getClawLeft() == SSValues.CLAW_LEFT_OPEN) {
                    upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
                    upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);
                } else {
                    upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
                    upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
                }
            }

            if (goToLastStoredPos.toTrue()) {
                driveMode = 2;
                drive.setSimpleMovePower(1);
                drive.setSimpleMoveTolerance(3, 3, Math.toRadians(3));
                drive.moveTo(drive.lastStoredPos, 100, () -> {
                    Action.buildSequence(update);
                    update.run();
                });
                driveMode = 0;
            }

            if (lockSlide.toTrue() && upper.getSequence() == SuperStructure.Sequences.ASCENT) {
                upper.lockSlide();
            }

            if(storeThisPos.toTrue()){
                drive.storeCurrentPos();
            }

            if(switchDrive.toTrue()){
                if(driveMode != 0){
                    toggleDriveMode(0);
                }else{
                    toggleDriveMode(1);
                }
            }
        }
    }

    private void drive_period() {
        if (upper != null) {
            if (driveMode == 0) {
                drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
            }else if (driveMode == 1){
                drive.setHeadingPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
            }
            drive.updateOdo();
        }
    }

    private void toggleDriveMode(int mode) {
        if (!drive.simpleMoveIsActivate) {
            driveMode = mode;
        }
    }

    // Logic updates with telemetry
    private void logic_period() {
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        XCYBoolean.bulkRead();
        count ++;
        telemetry.addData("Ticks since start: ", count);
        telemetry.addData("Ticks/second:",(double)count/(System.currentTimeMillis()/1000));
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        
        telemetry.update();
//        telemetry.addData("Arm Position: ", upper.getArmPosition());
//        telemetry.addData("Slide Position: ", upper.getSlidesPosition());
//        telemetry.addLine("");
//        telemetry.addData("Arm Power", upper.getArmPower());
//        telemetry.addData("Slide Power:", upper.getSlidePower());
//        telemetry.addLine("");
////        telemetry.addData("Arm Target Position", upper.getArmTargetPosition());
////        telemetry.addData("Slide Target Position", upper.getSlideTargetPosition());
//        telemetry.addData("Current Sequence", upper.getSequence());
//        telemetry.addData("Previous Sequence", upper.getPreviousSequence());
//        telemetry.addLine("");
////        telemetry.addData("Drive Mode", driveMode);
//        telemetry.addData("Action Stop?", Action.stopBuilding);
////            telemetry.addData("Bot Heading", Math.toDegrees(drive.getHeading()));
//        telemetry.addData("Touch Sensor Pressed?", upper.mTouchSensor.isPressed());
//        telemetry.addData("Last Stored Pose:", drive.getStoredPosAsString());
//        telemetry.addData("Current Pos", drive.getCurrentPoseAsString());
//        telemetry.addData("DriveMode: ", driveMode);
////        telemetry.addData("Slide Lock Position", upper.getSlideLockPosition());
//        telemetry.addLine(Action.showCurrentAction());
//        telemetry.update();
////        telemetry_M.update();
//
////        telemetry_M.addData("Slide Power:", upper.getSlidePower());
////        telemetry_M.addData("Arm Power", upper.getArmPower());
////        telemetry_M.update();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}