package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.*;

/**
 * 希望它不要爆掉...如果爆掉了就重启吧!
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                   O\  =  /O
 *                ____/`---'\____
 *              .'  \\|     |//  `.
 *             /  \\|||  :  |||//  \
 *            /  _||||| -:- |||||-  \
 *            |   | \\\  -  /// |   |
 *            | \_|  ''\---/''  |   |
 *            \  .-\__  `-`  ___/-. /
 *          ___`. .'  /--.--\  `. . __
 *       ."" '<  `.___\_<|>_/___.'  >'"".
 *      | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *      \  \ `-.   \_ __\ /__ _/   .-` /  /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *             佛祖保佑       永无BUG
 **/

@TeleOp(name = "16093 TeleOp")
public class TeleOp16093 extends LinearOpMode {
    NewMecanumDrive drive;
    SuperStructure upper;
    Pose2d current_pos;
    Runnable update;
    //Runnable update;

    // Modes for system control
    int driveMode = 0; // 0: POV mode; 1: Field-centric mode
    public static int slideMode=0;//1: setpower
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

        //  =====button assignments=====
        // Gamepad 1 button assignments

        XCYBoolean resetPos = new XCYBoolean(() -> gamepad1.left_stick_button);
        XCYBoolean resetOdo = new XCYBoolean(() -> gamepad1.a);
        XCYBoolean switchDrive = new XCYBoolean(() -> gamepad1.back);
        XCYBoolean releaseSample = new XCYBoolean(() -> gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0);
//        XCYBoolean intakeActive = new XCYBoolean(()-> gamepad1.right_bumper || gamepad1.left_bumper);
        XCYBoolean ascendingUp = new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean ascendingDown = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean forceStop = new XCYBoolean(() -> gamepad1.b);
//        XCYBoolean goToLastStoredPos = new XCYBoolean(()->gamepad1.y);
        XCYBoolean lockSlide = new XCYBoolean(()->gamepad1.y);


        // Gamepad 2 button assignments

        XCYBoolean intakeFar = new XCYBoolean(() -> gamepad2.dpad_up);
        XCYBoolean intakeNear = new XCYBoolean(() -> gamepad2.dpad_down);
        XCYBoolean releaseHigh = new XCYBoolean(() -> gamepad2.y);
        XCYBoolean releaseLow = new XCYBoolean(() -> gamepad2.a);
        XCYBoolean highChamberAim = new XCYBoolean(() -> gamepad2.right_bumper);
        XCYBoolean liftSlidesSlightly = new XCYBoolean(() -> gamepad2.left_bumper);
        XCYBoolean changeClaw = new XCYBoolean(() -> gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0);
        XCYBoolean wristHeightSwitch = new XCYBoolean(() -> gamepad2.right_stick_button);
        XCYBoolean armDownByPower = new XCYBoolean(()->gamepad2.options && !(gamepad2.back));
//        XCYBoolean resetSlide = new XCYBoolean(()->gamepad2.back && !(gamepad2.options));
        XCYBoolean resetFromBasketAuto = new XCYBoolean(()->gamepad2.back&&gamepad2.options);

        XCYBoolean resetArm = new XCYBoolean(()-> upper.getTouchSensorPressed());

        drive = new NewMecanumDrive(hardwareMap);

        update = ()->{
            logic_period();
            drive_period();
            drive.update();
            upper.update();
            if(forceStop.toTrue()){
                Action.stopBuilding = true;
            }
            if (forceStop.toFalse()) {
                Action.stopBuilding = false;
            }

            if (Action.actions.isEmpty() && resetArm.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN && (Math.abs(upper.getSlideError()) < 10 || upper.getSlideMode() == DcMotor.RunMode.RUN_USING_ENCODER)){
                upper.resetArmEncoder();
                upper.resetSlideEncoder();
            }

        };

        // Initialize and set up mecanum drive, starting position at (0,0,0)
        drive.setUpdateRunnable(update);
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));
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
//        Action.actions.clear();

        // Wait until play button is pressed

        waitForStart();


        // Set intake to default stop position and initialize operation mode
        upper.setIntake(SSValues.CONTINUOUS_STOP);
//        upper.startIntake();

        // Main control loop while op mode is active
        while (opModeIsActive() && !isStopRequested()) {

            if (Action.actions.isEmpty() && resetArm.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN && (Math.abs(upper.getSlideError()) < 10 || upper.getSlideMode() == DcMotor.RunMode.RUN_USING_ENCODER)){
                upper.resetArmEncoder();
                upper.resetSlideEncoder();
            }

            /////////////////////////////// OPERATIONS HANDLING ////////////////////////////////////

            // Accepts inputs only if mode is 0 (awaiting input)
            if (Action.actions.isEmpty()) {
                // Resets the position sequence if triggered by resetPos
                if (resetPos.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.RUN);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
//                    upper.stopIntake();
                    // Sequence actions based on last sequence
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE || upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN,900));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.ASCENT || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
                    }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
                    }
                }

                // High basket release sequences
                if (releaseHigh.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    drive.storeCurrentPos();

                    // Sequence actions for specific release sequences
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
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
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
                    }
                }

                // Intake sequences and similar conditional checks...
                if (intakeFar.toTrue()) {
                    drive.storeCurrentPos();
                    wristPos=0;
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN ){
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                        Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT));
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR){
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                    }else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
                        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                    }
                }
                if (intakeNear.toTrue()) {
                    drive.storeCurrentPos();
                    wristPos=0;
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
                    if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
                        Action.actions.add(new GrabAction(upper, SSValues.GRAB_DEFAULT));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_NEAR));
                    } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
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
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_TELEOP));
                }
                if(highChamberAim.toFalse() && upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE, 70));
                    Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_OPEN, SSValues.CLAW_RIGHT_OPEN,70));
                }

                //This part allows driver 2 to manually adjust the slide length by power if the upper.getSequence() is intake.
//                if(customSetSlide.toTrue()){
//
//                }
                if((Math.abs(gamepad2.left_stick_y) > 0.3) && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR)){
                    slideMode=1;
                    if(wristPos==1){
                        slideOpenloopConst=0.175;
                    }
                    else{
                        slideOpenloopConst=0.5;
                    }
                    if(gamepad2.left_stick_y > 0.3 && upper.getSlidesPosition() > 100){
                        upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, -gamepad2.left_stick_y*slideOpenloopConst);
                    }else if(gamepad2.left_stick_y < -0.3 && upper.getSlidesPosition() < SSValues.SLIDE_INTAKE_FAR-100) {
                        upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, -gamepad2.left_stick_y*slideOpenloopConst);
                    }else{
                        upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, 0);
                    }
                }else if (upper.getSlideMode() == DcMotor.RunMode.RUN_USING_ENCODER){
                    upper.setSlidesByPower(upper.getSlideTargetPosition(),0);
                }

                //Ascending
                if(ascendingUp.toTrue() && upper.getSequence() == SuperStructure.Sequences.LOW_BASKET){
                    upper.switchSequence(SuperStructure.Sequences.ASCENT);
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_ASCENT_UP));
                }
                if(ascendingDown.toTrue() && upper.getSequence() == SuperStructure.Sequences.ASCENT){
                    upper.setSlidesByP(SSValues.SLIDE_ASCENT_DOWN,1);
                }
                if(ascendingUp.toTrue() & upper.getSequence() != SuperStructure.Sequences.LOW_BASKET){
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_ASCENT_UP));
                }
                //This part allows driver 2 to manually move the arm down.
                if(armDownByPower.toTrue()) {
                    upper.switchSequence(SuperStructure.Sequences.RUN);
                    upper.setArmByPower(SSValues.ARM_DOWN,-1);
                }
                if(armDownByPower.toFalse()){
                    upper.setArmByPower(SSValues.ARM_DOWN,0);
                }
//                if(armDownByPower.toFalse()){
//                    upper.resetArmEncoder();
//                }
                if(gamepad2.back && !gamepad2.options) {
                    upper.setSlidesByPower(SSValues.SLIDE_MIN,-1);
                }

                if(resetFromBasketAuto.toTrue()){
                    upper.setWristPos(SSValues.WRIST_DEFAULT);
                    upper.setArmByPower(-SSValues.ARM_HANG1, -1);
                }
                if(resetFromBasketAuto.toFalse()){
                    upper.resetArmEncoder();
                    upper.resetSlideEncoder();
                    upper.setArmByPower(SSValues.ARM_DOWN, 0);
                    upper.setSlidesByPower(SSValues.SLIDE_MIN, 0);
                }
//
//                //This part turns off the power of the arm so that it stays in place better after the position is within acceptable range.
//                if(Math.abs(upper.getArmPosition()-upper.getArmTargetPosition()) < 20){
//                    upper.setArmByP(upper.getArmTargetPosition(), 0);
//                }

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


//                //Intake
//                if(intakeActive.toTrue()){
//                    intakeAct=true;
//                    upper.startIntake();
//                }
//
//                if(intakeActive.toFalse()){
//                    intakeAct=false;
//                    upper.stopIntake();
//                }

                if (gamepad1.right_bumper && (upper.getSequence()== SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence()== SuperStructure.Sequences.INTAKE_FAR || upper.getSequence()== SuperStructure.Sequences.HIGH_BASKET || upper.getSequence()== SuperStructure.Sequences.LOW_BASKET)) {
                    intakePosition = SSValues.CONTINUOUS_SPIN;
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                } else if (gamepad1.left_bumper && (upper.getSequence()== SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence()== SuperStructure.Sequences.INTAKE_FAR || upper.getSequence()== SuperStructure.Sequences.HIGH_BASKET || upper.getSequence()== SuperStructure.Sequences.LOW_BASKET)) {
                    intakePosition = SSValues.CONTINUOUS_SPIN_OPPOSITE;
                    upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
                }else{
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                }

                //Sample released when the arm is in the right place.
                if (releaseSample.toTrue()){
                    if(upper.getGrabPos() != SSValues.GRAB_OPEN){
                        upper.setGrabPos(SSValues.GRAB_OPEN);
                    }else{
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    }
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

//                if(gamepad1.y){
//                    drive.setSimpleMovePower(0.9);
//                    drive.setSimpleMoveTolerance(3,3,Math.toRadians(3));
//                    drive.moveTo(drive.lastStoredPos,1000);
//                }

                if(lockSlide.toTrue() && upper.getSequence()== SuperStructure.Sequences.ASCENT){
                    upper.lockSlide();
                }

                drive_period();
                logic_period();
                upper.update();




            }

            Action.buildSequence(this.update);
        }

    }



    /////////////////////////// SUPPORT METHODS ////////////////////////////

    // Drive control handling for mecanum drive based on selected mode

    private void drive_period() {
        if(upper!= null){
            if (driveMode == 0) {
                drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
            } else {
                drive.setHeadingPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
            }
            drive.updateOdo();
        }
    }

    // Logic updates with telemetry
    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.addData("Arm Position: ", upper.getArmPosition());
        telemetry.addData("Slide Position: ", upper.getSlidesPosition());
        telemetry.addLine("");
        telemetry.addData("Arm Power",upper.getArmPower());
        telemetry.addData("Slide Power:", upper.getSlidePower());
        telemetry.addLine("");
//        telemetry.addData("Arm Target Position", upper.getArmTargetPosition());
//        telemetry.addData("Slide Target Position", upper.getSlideTargetPosition());
        telemetry.addData("Current Sequence", upper.getSequence());
        telemetry.addData("Previous Sequence", upper.getPreviousSequence());
        telemetry.addLine("");
//        telemetry.addData("Drive Mode", driveMode);
//        telemetry.addData("Pinpoint Heading: ", drive.getHeading());
        telemetry.addData("Action Stop?", Action.stopBuilding);
        telemetry.addData("Touch Sensor Pressed?", upper.mTouchSensor.isPressed());
        telemetry.addData("Slide Lock Position", upper.getSlideLockPosition());
        telemetry.addLine(Action.showCurrentAction());
//        telemetry.addData("Intake left PWM", upper.controlLeft.getStatus());
//        telemetry.addData("Intake right PWM", upper.controlRight.getStatus());
        telemetry.update();
//        telemetry_M.update();

//        telemetry_M.addData("Slide Power:", upper.getSlidePower());
//        telemetry_M.addData("Arm Power", upper.getArmPower());
//        telemetry_M.update();
    }
}