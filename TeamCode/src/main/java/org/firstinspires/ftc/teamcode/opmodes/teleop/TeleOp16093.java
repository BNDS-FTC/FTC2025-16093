package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    Runnable update;
    //Runnable update;

    // Modes for system control
    int driveMode = 0; // 0: POV mode; 1: Field-centric mode
    public static int slideMode=0;//1: setpower
    public static int armMode = 0;//1: setpower
    int wristPos=0;//0:up;1:down
    boolean intakeAct = false;
    double slideOpenloopConst = 0.3;

    double intakePosition = SSValues.CONTINUOUS_STOP; // Intake servo initial position

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize SuperStructure with periodic functions for logic and drive control
        upper = new SuperStructure(
                this,
                () -> {}, 0);

        update = ()->{
            logic_period();
            drive_period();
            upper.update();
        };

        // Initialize and set up mecanum drive, starting position at (0,0,0)
        drive = new TeleOpDrive();
        drive.setUp(hardwareMap);
        //  =====button assignments=====
        // Gamepad 1 button assignments

        XCYBoolean resetPos = new XCYBoolean(() -> gamepad1.left_stick_button);
        XCYBoolean resetOdo = new XCYBoolean(() -> gamepad1.a);
        XCYBoolean switchDrive = new XCYBoolean(() -> gamepad1.back);
        XCYBoolean releaseSample = new XCYBoolean(() -> gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0);
        XCYBoolean intakeActive = new XCYBoolean(()-> gamepad1.right_bumper || gamepad1.left_bumper);
        XCYBoolean ascendingUP = new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean ascendingDOWN = new XCYBoolean(()->gamepad1.dpad_down);


        // Gamepad 2 button assignments

        XCYBoolean intakeFar = new XCYBoolean(() -> gamepad2.dpad_up);
        XCYBoolean intakeNear = new XCYBoolean(() -> gamepad2.dpad_down);
        XCYBoolean releaseHigh = new XCYBoolean(() -> gamepad2.y);
        XCYBoolean releaseLow = new XCYBoolean(() -> gamepad2.a);
        XCYBoolean highChamberAim = new XCYBoolean(() -> gamepad2.right_bumper);
        XCYBoolean liftSlidesSlightly = new XCYBoolean(() -> gamepad2.left_bumper);
        XCYBoolean changeClaw = new XCYBoolean(() -> gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0);
        XCYBoolean wristHeightSwitch = new XCYBoolean(() -> gamepad2.right_stick_button);
        XCYBoolean armDownByPower = new XCYBoolean(()->gamepad2.options);


        // =====Initial setup for upper mechanisms to default positions=====


        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.1);
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
            if (Action.actions.isEmpty()) {
                if (upper.getTouchSensorPressed()&&upper.getSequence()== SuperStructure.Sequences.RUN){
                    upper.resetArmEncoder();
                    upper.resetSlideEncoder();
                }
                // Resets the position sequence if triggered by resetPos
                if (resetPos.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.RUN);
                    // Sequence actions based on last sequence
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.HANG || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT, 300));
                    }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT,200));
                    }
                }

                // High basket release sequences
                if (releaseHigh.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);

                    // Sequence actions for specific release sequences
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 300));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }

                // Intake sequences and similar conditional checks...
                if (intakeFar.toTrue()) {
                    wristPos=0;
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN ){
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR){
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                    }else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                    }
                }
                if (intakeNear.toTrue()) {
                    wristPos=0;
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
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
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE || upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        Action.actions.add(new SlideAction(upper, 0));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                    }
                }

                if(liftSlidesSlightly.toTrue() && upper.getSequence() == SuperStructure.Sequences.LOW_BASKET){
                    upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
                }

                //To place the specimen on the chamber, driver 2 presses the right bumper continuously until it can be released.
                if (highChamberAim.toTrue() && upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
                    Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM));
                }
                if(highChamberAim.toFalse() && upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE, 70));
                    Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_OPEN, SSValues.CLAW_RIGHT_OPEN,70));
                }

                //This part allows driver 2 to manually adjust the slide length by power if the upper.getSequence() is intake.
                if((Math.abs(gamepad2.left_stick_y) > -0.1) && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR)){
                    if(intakeAct){
                        slideOpenloopConst=0.2;
                    }
                    else{
                        slideOpenloopConst=0.4;
                    }
                    if(gamepad2.left_stick_y > 0 && upper.getSlidesPosition() > 50){
                        slideMode=1;
                        upper.setSlidesByPower(-gamepad2.left_stick_y*slideOpenloopConst);
                    }else if(gamepad2.left_stick_y < 0.1 && upper.getSlidesPosition() < SSValues.SLIDE_INTAKE_FAR+50){
                        slideMode=1;
                        upper.setSlidesByPower(-gamepad2.left_stick_y*slideOpenloopConst);
                    }
                }else{
                    slideMode=0;
                }

                //Ascending
                if(ascendingUP.toTrue()){
                    upper.switchSequence(SuperStructure.Sequences.ASCENT);
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_ASCENT_UP));
                }
                if(ascendingDOWN.toTrue() && upper.getSequence() == SuperStructure.Sequences.ASCENT){
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_ASCENT_DOWN));
                }
                //This part allows driver 2 to manually move the arm down.
                if(gamepad2.options) {
                    upper.setArmByPower(-1);
                }
                if(armDownByPower.toFalse()){
                    upper.setArmByPower(0);
                    upper.setArmByP(0,1);
                }
                if(gamepad2.back) {
                    upper.setSlidesByPower(-1);
                }
                if(armDownByPower.toFalse()){
                    upper.setSlidesByPower(0);
                    upper.setSlidesByP(0,1);
                }

                //This part turns off the power of the arm so that it stays in place better after the position is within acceptable range.
                if(Math.abs(upper.getArmPosition()-upper.getArmTargetPosition()) < 20){
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
                if(intakeActive.toTrue()){
                    intakeAct=true;
                    upper.startIntake();
                }

                if(intakeActive.toFalse()){
                    intakeAct=false;
                    upper.stopIntake();
                }

                if (gamepad1.right_bumper) {
                    intakePosition = SSValues.CONTINUOUS_SPIN;
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                } else if (gamepad1.left_bumper) {
                    intakePosition = SSValues.CONTINUOUS_SPIN_OPPOSITE;
                    upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
                }

                //Sample released when the arm is in the right place.
                if (releaseSample.toTrue()){
                    upper.setGrabPos(SSValues.GRAB_OPEN);
                }

                if ((upper.getSequence()== SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence()== SuperStructure.Sequences.INTAKE_FAR)&&wristHeightSwitch.toTrue()){
                    if (wristPos==0){
                        upper.setWristPos(SSValues.WRIST_INTAKE);
                        wristPos=1;
                    }else{
                        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                        wristPos=0;
                    }
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

                if (slideMode==0){
                    upper.update();
                }



            }

            Action.buildSequence(this.update);
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
        telemetry.addData("Slide Velocity", upper.getSlideVelocity());
        telemetry.addData("Slide Power:", upper.getSlidePower());
        telemetry.addData("Arm Power",upper.getArmPower());
        telemetry.addData("Current Sequence", upper.getSequence());
        telemetry.addData("Previous Sequence", upper.getPreviousSequence());
        telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("Intake Mode", intakePosition);
        telemetry.addData("Pinpoint Heading: ", drive.getHeading());
        telemetry.update();

        telemetry_M.addData("Slide Power:", upper.getSlidePower());
        telemetry_M.addData("Arm Power", upper.getArmPower());
        telemetry_M.update();
    }
}