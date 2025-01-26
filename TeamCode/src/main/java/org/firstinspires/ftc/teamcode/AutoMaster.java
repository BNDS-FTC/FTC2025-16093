package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.CancellableFinishConditionActionGroup;
import org.firstinspires.ftc.teamcode.actions.FinishConditionActionGroup;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.ArmAction;
import org.firstinspires.ftc.teamcode.actions.GrabAction;
import org.firstinspires.ftc.teamcode.actions.ParallelActionGroup;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.actions.WristAction;

@Config
public abstract class AutoMaster extends LinearOpMode {

    private NewMecanumDrive drive;
    protected SuperStructure upper;
    protected Runnable update;

    Pose2d startPos;

    double oldTime = 0;
    int loopCount = 0;
    static long startTime;


    protected void initHardware(Pose2d start) throws InterruptedException{
        startPos = start;
        startTime = System.currentTimeMillis();
        Action.clearActions();

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new NewMecanumDrive(hardwareMap);
        drive.resetOdo();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.update();
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(10));

        telemetry.addLine("init: superstructure");
        telemetry.update();
        upper = new SuperStructure(
                this,
                () -> {
                }, SSValues.AUTO_ARM_OFFSET);

        XCYBoolean resetArm = new XCYBoolean(()-> upper.getTouchSensorPressed());


        update = ()->{
            drive.update();
            upper.update();
            XCYBoolean.bulkRead();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            loopCount ++;
            telemetry.addData("Loops since start: ", loopCount);
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

            telemetry.addData("In Distress?", drive.simpleMoveInDistress);
//            telemetry.addData("Start Time: ", drive.startTime);
//            telemetry.addData("Time since last MoveTo: ", drive.millisSinceMoveTo);
            telemetry.addData("Current Pose", drive.getCurrentPoseAsString());
            telemetry.addData("Target Pose",(drive.simpleMoveIsActivate)? drive.getSimpleMovePosition().toString() : "SimpleMove not activated");
            telemetry.addData("Arm Position: ", upper.getArmPosition());
            telemetry.addData("Arm Target: ", upper.getArmTargetPosition());
            telemetry.addData("Slide Target: ", upper.getSlideTargetPosition());
            telemetry.addData("Slide Error: ", upper.getSlideError());
            telemetry.addData("Slide Power: ", upper.getSlidePower());
            telemetry.addData("Current Sequence: ",upper.getSequence());
            telemetry.addData("Current Time: ", System.currentTimeMillis()-startTime);

            if(upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                telemetry.addData("Detected Sample Color", upper.colorOfSample());
                telemetry.addData("Color Raw Values", upper.getColorRGBAValues(10).toString());
//            telemetry.addData("Is there a sample?", upper.colorSensorCovered());
            }

            telemetry.addLine(Action.showCurrentAction());
            telemetry.update();
//            if (Action.actions.isEmpty()) {
//                if (resetArm.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN) {
//                    upper.resetArmEncoder();
//                    upper.resetSlideEncoder();
//                }
//            }
            //TODO: THIS DOESN'T WORK
//            if(drive.simpleMoveInDistress){
//                prepareForTeleOpWithoutMoving();
//            }
            if(isStopRequested()){
                Action.stopBuilding = true;
            }
        };

        drive.setUpdateRunnable(update);
        drive.setSwitchDrivePIDCondition(upper.slideTooHigh);

        upper.resetSlide();
        upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);//Maybe we should test this!
        upper.setArmByP(SSValues.ARM_DOWN, 0.5);
//        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);

        telemetry.addLine("init: trajectory");
        telemetry.update();


    }

    protected void setStartTime(){
        startTime = System.currentTimeMillis();
    }

    ///////////////////////////////////BLUE BASKET///////////////////////////////////////////////
    protected void reset(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        // Sequence actions based on last sequence
        if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
            upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
        } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.ASCENT || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
            upper.setGrabPos(SSValues.GRAB_DEFAULT);
            Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
            Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        }
        Action.buildSequence(update);
    }

    protected void newResetFromHighChamber(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 600));
    }
    protected void newResetCompletelyFromHighChamber(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 700));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,800));
        Action.buildSequence(update);
    }
    protected Pose2d yellowPose;
    protected void resetAfterBlueBasketAndMoveToIntake(double xOffset, double headingOffset){
        yellowPose = new Pose2d(47.5+xOffset, 47, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.75);
        drive.moveTo(new Pose2d(50, 45, Math.toRadians(-135)), 20);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 100));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        drive.moveTo(yellowPose, 600, ()->Action.buildSequence(update));
    }

    protected void resetAfterRedBasketAndMoveToIntake(double xOffset, double headingOffset){
        yellowPose = new Pose2d(-47.5-xOffset, -46.5, Math.toRadians(90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.75);
        drive.moveTo(new Pose2d(-50, -50, Math.toRadians(45)), 0);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 100));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        drive.moveTo(yellowPose, 500, ()->Action.buildSequence(update));
    }

    protected void expResetAfterBlueBasketAndMoveToIntake(double xOffset, double yOffset, double headingOffset){
        yellowPose = new Pose2d(47.5+xOffset, 47+yOffset, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.5);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 50));
        Action.buildSequence(update);
        drive.moveTo(new Pose2d(50, 50, Math.toRadians(-135)), 0);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        drive.moveTo(yellowPose, 200, ()->Action.buildSequence(update));
    }

    protected void expResetAfterRedBasketAndMoveToIntake(double xOffset, double headingOffset){
        yellowPose = new Pose2d(-47.5-xOffset, -46.5, Math.toRadians(90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.85);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 50));
        Action.buildSequence(update);
        drive.moveTo(new Pose2d(50, 50, Math.toRadians(-135)), 0);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        drive.moveTo(yellowPose, 400, ()->Action.buildSequence(update));
    }

    protected void prepareForTeleOpBlue(){
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,30));
        drive.moveTo(new Pose2d(-48, 55, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
//        upper.resetSlide();
//        upper.setGrabPos(SSValues.GRAB_CLOSED);
//        upper.setWristPos(SSValues.WRIST_DEFAULT);
//        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);//Maybe we should test this!
//        upper.setArmByP(SSValues.ARM_DOWN -SSValues.AUTO_ARM_OFFSET, 0.5);
//        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
//        sleep(500);
    }

    protected void prepareForTeleOpRed() {
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 30));
        drive.moveTo(new Pose2d(48, -55, Math.toRadians(90)), 100, () -> Action.buildSequence(update));
    }

    protected void expPrepareForTeleOpBlue() {
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 30));
        drive.setSimpleMovePower(1);
        drive.moveTo(new Pose2d(-50, 55, Math.toRadians(135)), 100, () -> Action.buildSequence(update));

    }

    protected void expPrepareForTeleOpRed() {
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 30));
        drive.setSimpleMovePower(1);
        drive.moveTo(new Pose2d(50, -55, Math.toRadians(-45)), 100, () -> Action.buildSequence(update));
    }

    protected void prepareForTeleOpWithoutMoving(){
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,30));
        Action.buildSequence(update);
    }

    protected void moveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD,800));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(-10+xOffset, 41.5, Math.toRadians(91)), 150,()->Action.buildSequence(update));
//        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
//        drive.setSimpleMovePower(0.7);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(-10+xOffset, 35, Math.toRadians(90)), 100,()->Action.buildSequence(update));
    }

    protected void moveToRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD,800));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(10-xOffset, -42, Math.toRadians(-89)), 100,()->Action.buildSequence(update));
//        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
//        drive.setSimpleMovePower(0.7);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(10-xOffset, -35, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
    }

    protected void expMoveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.95);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
//        //以下应该保留，先注释掉看看效果
//        drive.moveTo(new Pose2d(-10+xOffset, 42, Math.toRadians(90)), 100,()->Action.buildSequence(update));
//        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
//        drive.setSimpleMovePower(0.8);
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(-10+xOffset, 35, Math.toRadians(90)), 100,()->Action.buildSequence(update));
    }

    protected void expMoveToRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD,1400));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(10-xOffset, -42, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
        drive.setSimpleMovePower(0.8);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(10-xOffset, -35, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
    }

    protected void VexpMoveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.95);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-58,57,Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-40,57), Math.toRadians(0))
                .splineTo(new Vector2d(-10+xOffset,42), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj);

//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
//        drive.moveTo(new Pose2d(-10+xOffset, 42, Math.toRadians(90)), 100,()->Action.buildSequence(update));
        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
        drive.setSimpleMovePower(0.8);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(-10+xOffset, 35, Math.toRadians(90)), 100,()->Action.buildSequence(update));
    }

    protected void firstMoveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.6);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 600));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(-10+xOffset, 45, Math.toRadians(90)),0,()->Action.buildSequence(update));
        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);
        drive.moveTo(new Pose2d(-10+xOffset, 36, Math.toRadians(90)), 30);
    }

    protected void firstMoveToRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.5);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 600));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(10-xOffset, -55, Math.toRadians(-90)),0,()->Action.buildSequence(update));
        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
        drive.moveTo(new Pose2d(10-xOffset, -36, Math.toRadians(-90)), 30);
    }

    protected void expFirstMoveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.8);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 700));
        Action.buildSequence(update);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(-10+xOffset, 36, Math.toRadians(90)),0,()->Action.buildSequence(update));
    }

    protected void expFirstMoveToRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.6);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 700));
        Action.buildSequence(update);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,40));
        drive.moveTo(new Pose2d(10-xOffset, -36, Math.toRadians(-90)),0,()->Action.buildSequence(update));
    }

    protected void newFirstMoveToBlueChamberPlace(){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
//        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 800));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_LONGER,20));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,20));
        drive.moveTo(new Pose2d(-10, 39.3, Math.toRadians(90)),50,()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void newBlueChamberPlace(double xOffset, double yPlaceOffset){
        drive.setSimpleMoveTolerance(5,5, Math.toRadians(7));
        drive.setSimpleMovePower(0.7);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        upper.setWristPos(SSValues.WRIST_HIGH_CHAMBER);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 700));
//        Action.buildSequence(update);
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,50),new ArmAction(upper, SSValues.ARM_UP, 30)));
        drive.moveTo(new Pose2d(-10+xOffset, 45, Math.toRadians(90)),50,()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-10+xOffset, 37.2, Math.toRadians(90)),50);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,170));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void newParkFromBlueChamber(){
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,400));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN,100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,20));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(-20, 48, Math.toRadians(135)), 0, ()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,100));
        Action.buildSequence(update);
    }

    protected void newerBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED, 100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER,20));
//        drive.moveTo(new Pose2d(-10+xOffset, 37, Math.toRadians(90)),300,()->Action.buildSequence(update));
        Pose2d trajStartPos = drive.getPoseEstimate();
        Trajectory traj = drive.trajectoryBuilder(trajStartPos)
                .splineToConstantHeading(new Vector2d(trajStartPos.getX(), trajStartPos.getY()-1), -90)
                .splineTo(new Vector2d(trajStartPos.getX()+40+xOffset, trajStartPos.getY()-20),-90)
                .build();
//        drive.followTrajectory(traj, ()->Action.buildSequence(update));

        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,100));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void highChamberPlace(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,100));
//        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_OPEN, SSValues.CLAW_RIGHT_OPEN,70));
        Action.buildSequence(update);
    }

    protected void firstIntakeSpecimenFromGround(double xOffset, double yOffset){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,400));
        Action.actions.add(new SlideAction(upper, (int)(SSValues.SLIDE_INTAKE_FAR*0.8),100, 0.4));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,20));
        drive.moveTo(new Pose2d(-21+xOffset, 45+yOffset, Math.toRadians(135)), 40, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,20,0.15),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);}));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED));
        Action.buildSequence(update);
    }

    protected void intakeSpecimenFromGround(double xOffset, double yOffset){
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,600));
        Action.actions.add(new SlideAction(upper, (int)(SSValues.SLIDE_INTAKE_FAR*0.75),10, 0.8));
        drive.moveTo(new Pose2d(-20+xOffset, 41+yOffset, Math.toRadians(135)), 10, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,20,0.15),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);}));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.3));
//        Action.buildSequence(update);
//        delay(40);
//        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }


    protected void intakeSpecimenFromGroundWithoutColor(double xOffset, double yOffset){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,400));
        Action.actions.add(new SlideAction(upper, (int)(SSValues.SLIDE_INTAKE_FAR*0.8),100, 0.6));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,20));
        drive.moveTo(new Pose2d(-20+xOffset, 45+yOffset, Math.toRadians(135)), 20, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.3));
        delay(120);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void newFirstMoveToRedChamberPlace(){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
//        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 600));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_LONGER,20));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        drive.moveTo(new Pose2d(10, -40, Math.toRadians(90)),20,()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,100));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void newRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER,600),new ArmAction(upper, SSValues.ARM_UP, 700)));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,10));
        drive.moveTo(new Pose2d(-10+xOffset, 37, Math.toRadians(90)),20,()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,100));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void newParkFromRedChamber(){
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,400));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN,100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,20));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(-20, 48, Math.toRadians(135)), 0, ()->Action.buildSequence(update));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,100));
        Action.buildSequence(update);
    }



    protected void intakeSpecimenFromWall(double xOffset, double yOffset){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_SLIGHTLY_HIGHER,100));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE_SPECIMEN));
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.95);
        drive.moveTo(new Pose2d(-37.5+xOffset, 55+yOffset, Math.toRadians(90)), 100,()->Action.buildSequence(update));
        delay(120);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }




    ///////////////////////////////////BLUE//////////////////////////////////////////

//    protected void moveToBlueChamberAim(){
//        drive.setSimpleMoveTolerance(2,0.7, Math.toRadians(5));
//        drive.setSimpleMovePower(0.9);
//        drive.moveTo(new Pose2d(0, 43, Math.toRadians(90)), 500);
//        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
//        Action.buildSequence(update);
//    }




    protected Pose2d blueBasket = new Pose2d(52.3, 54, Math.toRadians(-135));
    protected Pose2d redBasket = new Pose2d(-52.3, -55.3, Math.toRadians(45));
    protected void firstPutBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
//        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,0));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(52, 53, Math.toRadians(-135)), 600,()->Action.buildSequence(update));
//        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        delay(100);
    }

    protected void firstPutRedBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
//        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,0));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-53, -54, Math.toRadians(45)), 800,()->Action.buildSequence(update));
//        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        delay(200);
    }

    protected void expFirstPutBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.65);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 700));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,0));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(53, 53, Math.toRadians(-125)), 250,()->Action.buildSequence(update));
//        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
    }

    protected void expFirstPutRedBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
//        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 500));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,0));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-53, -54, Math.toRadians(45)), 400,()->Action.buildSequence(update));
//        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        delay(100);
    }

    protected void putBlueBasketFromGround(double xOffset, double yOffset, double simpleMovePowerChange){
        blueBasket = new Pose2d(51.3+xOffset, 54+yOffset, Math.toRadians(-125));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.4 + simpleMovePowerChange);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 900));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,400));
        drive.moveTo(blueBasket, 150,()->Action.buildSequence(update));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        sleep(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putBlueBasketFromSubmersible(double xOffset, double yOffset, double simpleMovePowerChange){
        blueBasket = new Pose2d(51.3+xOffset, 54+yOffset, Math.toRadians(-130));
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.4 + simpleMovePowerChange);
//        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 400));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 900));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        drive.moveTo(new Pose2d(33, 20, Math.toRadians(-135)),0, ()->Action.buildSequence(update));
        drive.setSimpleMovePower(0.55);
        Action.actions.add(new ParallelActionGroup((new ArmAction(upper, SSValues.ARM_UP, 200)), (new SlideAction(upper, SSValues.SLIDE_MAX, 50))));
        drive.setSimpleMoveTolerance(1.5, 1.5, Math.toRadians(7));
        drive.moveTo(blueBasket, 120,()->Action.buildSequence(update));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,400));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(()->{update.run(); drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);});
        delay(100);
//        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
//        sleep(100);
//        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putRedBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
//        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600);
        Action.actions.add(new GrabAction(upper, SSValues.AUTO_GRAB_CLOSED, 20));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 400));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,600));
        drive.moveTo(redBasket, 400,()->Action.buildSequence(update));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        sleep(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void hangFromBlueBasket(){
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 30));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 400));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 5));
        drive.moveTo(new Pose2d(40,10, Math.toRadians(-90)), 200, ()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 100));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_LONGER, 400));
        drive.moveTo(new Pose2d(40,10, Math.toRadians(180)), 200, ()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(20,10, Math.toRadians(180)), 200, ()->Action.buildSequence(update));
    }

    protected void ExpHangFromBlueBasket(){
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(7));
        drive.setSimpleMovePower(1);
        Action.actions.add(new ArmAction(upper,SSValues.ARM_HANG1,300));
        Action.actions.add(new SlideAction(upper,SSValues.SLIDE_MIN,30));
        drive.moveTo(new Pose2d(40,13,180),100,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_LONGER, 400));
        drive.moveTo(new Pose2d(23,13,180),100,()->Action.buildSequence(update));
    }

    protected void hangFromRedBasket(){
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 30));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 400));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 5));
        drive.moveTo(new Pose2d(-40,-10, Math.toRadians(90)), 200, ()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 100));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_LONGER, 400));
        drive.moveTo(new Pose2d(-40,-10, Math.toRadians(0)), 200, ()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-20,-10, Math.toRadians(0)), 200, ()->Action.buildSequence(update));
    }

    protected void pushTwoBlueSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        drive.moveTo(new Pose2d(-36, 40, Math.toRadians(90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(-36, 15, Math.toRadians(90)), 0,()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-47.5, 15, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-47.5, 52, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-44, 15, Math.toRadians(-90)), 0);
        drive.moveTo(new Pose2d(-55, 15, Math.toRadians(-90)), 0); //-55
        drive.moveTo(new Pose2d(-56.5, 52, Math.toRadians(-90)), 0); //-56.5
    }

    // Method to move the robot with proportional deceleration and motor power adjustment
    private void moveToWithDeceleration(Pose2d target, double speed) {
        // Get the current position of the robot
        Pose2d currentPose = drive.getPoseEstimate();

        // Calculate the distance to the target
        double targetY = target.getY();
        double currentY = currentPose.getY();

        double distanceToTarget = Math.abs(targetY - currentY);

        // Define the maximum speed
        double maxSpeed = speed;  // You can adjust the max speed here

        // Calculate the speed using proportional deceleration based on remaining distance
        double adjustedSpeed = calculateSpeed(distanceToTarget, maxSpeed);

        // Convert the adjusted speed into motor power
        double motorPower = speedToMotorPower(adjustedSpeed);

        // Move to the target with adjusted motor power
        drive.setMotorPowers(motorPower, motorPower,motorPower,motorPower);
        drive.moveTo(new Pose2d(target.getX(),
                target.getY(),
                target.getHeading()),0);
    }

    // Function to calculate the robot's speed based on distance to the target
    private double calculateSpeed(double distanceToTarget, double maxSpeed) {
        // Proportional deceleration: speed decreases as distance decreases
        double speed = maxSpeed * (distanceToTarget / 5);  // 5 is the maximum distance to decelerate (you can adjust this value)

        // Ensure the speed doesn't go below a minimum threshold
        if (speed < 0.1) {  // You can adjust the minimum speed
            speed = 0.1;
        }

        return speed;
    }

    // Convert speed to motor power (scale to -1 to 1 range)
    private double speedToMotorPower(double speed) {
        // Assuming the max speed is normalized to 1.0, and we want the motor power to be proportional to this speed
        // Ensure the motor power is within -1.0 and 1.0
        double motorPower = speed;  // Since speed is normalized between 0 and maxSpeed, motor power can directly match it

        // Optional: Apply a deadzone to avoid jittering at low speeds (you can tune this value)
        if (Math.abs(motorPower) < 0.1) {
            motorPower = 0;  // Deadzone to prevent small motor movements
        }

        return motorPower;
    }

    protected void pushTwoRedSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        drive.moveTo(new Pose2d(36, -40, Math.toRadians(-90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(36, -15, Math.toRadians(-90)), 0,()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(47.5, -15, Math.toRadians(-90)), 0);
        drive.moveTo(new Pose2d(47.5, -52, Math.toRadians(-90)), 0);
        drive.moveTo(new Pose2d(44, -15, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(55, -15, Math.toRadians(90)), 0);//55
        drive.moveTo(new Pose2d(56.5, -52, Math.toRadians(90)), 0);//56.5
    }

    protected void expPushTwoBlueSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,10));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        drive.moveTo(new Pose2d(-36, 40, Math.toRadians(90)), 0,()->Action.buildSequence(update));
//        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(-36, 15, Math.toRadians(-135)), 0);
//        drive.moveTo(new Pose2d(47.5, -15, Math.toRadians(-90)), 0);
        drive.moveTo(new Pose2d(-50, 52, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-50, 15, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-55, 15, Math.toRadians(90)), 0);//55
        drive.moveTo(new Pose2d(-56.5, 52, Math.toRadians(90)), 0);//56.5
    }

    protected void expPushTwoRedSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        drive.moveTo(new Pose2d(36, -40, Math.toRadians(-90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(36, -15, Math.toRadians(-45)), 0,()->Action.buildSequence(update));
//        drive.moveTo(new Pose2d(47.5, -15, Math.toRadians(-90)), 0);
        drive.moveTo(new Pose2d(47, -52, Math.toRadians(-90)), 0);
        drive.moveTo(new Pose2d(47, -15, Math.toRadians(-90)), 0);
        drive.moveTo(new Pose2d(55, -15, Math.toRadians(-90)), 0);//55
        drive.moveTo(new Pose2d(56.5, -52, Math.toRadians(-90)), 0);//56.5
    }

    protected void VexpPushTwoRedSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        drive.moveTo(new Pose2d(36, -40, Math.toRadians(-90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(36, -15, Math.toRadians(-45)), 0,()->Action.buildSequence(update));
        drive.moveWithDrift(new Pose2d(49, -52, Math.toRadians(-90)),new Pose2d(49, -15, Math.toRadians(-90)),new Pose2d(57, -15, Math.toRadians(-90)),new Pose2d(57, -52, Math.toRadians(-90)));
//        drive.moveTo(new Pose2d(50, -52, Math.toRadians(-90)), 0);
//        drive.moveTo(new Pose2d(50, -15, Math.toRadians(-90)), 0);
//        drive.moveTo(new Pose2d(55, -15, Math.toRadians(-90)), 0);
//        drive.moveTo(new Pose2d(56.5, -52, Math.toRadians(-90)), 0);
    }

    protected void VexpPushTwoBlueSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(0.9);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        drive.moveTo(new Pose2d(-36, 40, Math.toRadians(90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        drive.moveTo(new Pose2d(-38, 15, Math.toRadians(90)), 20,()->Action.buildSequence(update));
        drive.moveWithDrift(new Pose2d(-46, 17, Math.toRadians(90)),
                new Pose2d(-46, 48, Math.toRadians(90)),
                new Pose2d(-46, 17, Math.toRadians(90)),
                new Pose2d(-54, 17, Math.toRadians(90)),
                new Pose2d(-55, 47, Math.toRadians(90)),
                new Pose2d(-54, 17, Math.toRadians(90)),
                new Pose2d(-61, 17, Math.toRadians(90)),
                new Pose2d(-61, 42, Math.toRadians(90)));
    }

    protected void intakeThreeBlueSamples(){
        getBlueSamplesFromGround(-0.5,0,-2);
        throwBlueSamplesBehind(-10);

        getBlueSamplesFromGround(-11,1,-2);
        throwBlueSamplesBehind(0);

        moveAndIntakeLastChamberSampleBlue();
        throwBlueSamplesBehind(0);
    }

    protected void newPushTwoBlueSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        Action.buildSequence(update);
        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR, 200));
        drive.moveTo(new Pose2d(-17, 43, Math.toRadians(225)), 0,()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-17, 43, Math.toRadians(135)),0);

    }

    protected void getYellowSamples(){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.7);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,0));
        Action.actions.add(new SlideAction(upper, (int)0.9*SSValues.SLIDE_AUTO_INTAKE_YELLOW,20,1));
//        Action.buildSequence(()->drive.moveTo(new Pose2d(47.5+xOffset, 46, Math.toRadians(-90)), 350));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_YELLOW,10, 0.4));
        Action.buildSequence(update);
        sleep(30);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void expGetYellowSamples(){
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.65);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,0));
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_YELLOW,20,0.4),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);}));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setGrabPos(SSValues.GRAB_CLOSED);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.buildSequence(update);
    }

    private Pose2d bluePose;

    protected void getBlueSamplesFromGround(double xOffset, double yOffset, double headingOffset){
        bluePose = new Pose2d(-47.5+xOffset, 47+yOffset, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.5);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 500));
        drive.moveTo(bluePose, 100, ()->Action.buildSequence(update));
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,20));
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_YELLOW,20,0.3),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);}));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }
    protected void throwBlueSamplesBehind(double xOffset){
        drive.setSimpleMovePower(0.8);
        upper.switchSequence(SuperStructure.Sequences.LOW_BASKET);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 500));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,0));
        drive.moveTo(new Pose2d(bluePose.getX()+xOffset, bluePose.getY(), bluePose.getHeading()), 20, ()->Action.buildSequence(update));
        upper.setGrabPos(SSValues.GRAB_OPEN);
    }

    protected void getSamplesFromSubmersibleBlue(double offset){

        resetAndGoToBlueSubmersible(300,offset);

        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.25),
                ()-> upper.colorOfSample().equals("blue")||upper.colorOfSample().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);},
                ()->{drive.moveTo(new Pose2d(27.1,12, Math.toRadians(180)),10);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    tryAgainAtBlueSubmersible();}));
        Action.buildSequence(()->{update.run();drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);});
        upper.setIntake(SSValues.CONTINUOUS_STOP);

    }

    protected void getSamplesFromSubmersibleBlueWithEmergencyAscent(double offset){
        CancellableFinishConditionActionGroup grabBlueFromSubmersible = new CancellableFinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.2),
                ()->upper.colorOfSample().equals("blue")||upper.colorOfSample().equals("yellow"),
                ()->(System.currentTimeMillis()-startTime>27*1000),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{Action.clearActions();
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1));
                    drive.moveTo(new Pose2d(20,10, Math.toRadians(180)),100,()->Action.buildSequence(update));
                    while(opModeIsActive()){}});

//        FinishConditionActionGroup hangIfNearEnd = new FinishConditionActionGroup(grabBlueFromSubmersible,
//                ()-> System.currentTimeMillis()-startTime > 29500,
//                ()->{Action.clearActions();
//                    Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1));
//                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
//                    Action.buildSequence(update);
//                    while (opModeIsActive()) {}},
//                ()->{});

        resetAndGoToBlueSubmersible(50,offset);

        Action.actions.add(grabBlueFromSubmersible);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);

    }


    private void resetAndGoToBlueSubmersible(int correctTime, double offset){
        drive.setSimpleMoveTolerance(5,5,Math.toRadians(5));
        drive.setSimpleMovePower(1);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 250));
        drive.moveTo(new Pose2d(blueBasket.getX(), blueBasket.getY(), -140),0, ()->Action.buildSequence(update));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,300));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(40, 11+offset, Math.toRadians(-135)), 0, () -> Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 400));
        drive.moveTo(new Pose2d(27.1,9+offset, Math.toRadians(-140)), 0, ()->Action.buildSequence(update));
        upper.setWristPos(SSValues.WRIST_INTAKE);
        drive.moveTo(new Pose2d(27.1,9+offset, Math.toRadians(180)), correctTime, () -> {
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                }
        );
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void tryAgainAtBlueSubmersible(){
        FinishConditionActionGroup reachAgainAtBlueSubmersible = new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.24),
                ()->upper.colorOfSample().equals("blue")||upper.colorOfSample().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->upper.setIntake(SSValues.CONTINUOUS_STOP));

        FinishConditionActionGroup hangIfNearEnd = new FinishConditionActionGroup(reachAgainAtBlueSubmersible,
                ()-> System.currentTimeMillis()-startTime > 29500,
                ()->{Action.clearActions();
                    Action.actions.add(new ArmAction(upper, SSValues.ARM_HANG1));
                    Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER));
                    Action.buildSequence(update);
                    while (opModeIsActive()) {}},
                ()->{});

        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        drive.moveTo(new Pose2d(28.5,6, Math.toRadians(180)), 50, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(hangIfNearEnd);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }


    protected void parkAtBlueObservationFromBasket(){
        drive.setSimpleMoveTolerance(5,5,Math.toRadians(3));
        drive.setSimpleMovePower(0.99);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-38, 60, Math.toRadians(0)), 0));
    }
    protected void parkAtRedObservationFromBasket(){
        drive.setSimpleMoveTolerance(5,5,Math.toRadians(3));
        drive.setSimpleMovePower(0.99);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        Action.buildSequence(()->drive.moveTo(new Pose2d(38, -60, Math.toRadians(180)), 0));
    }

    protected void simpleParkAtObservation(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
        drive.setSimpleMovePower(0.99);
        Action.buildSequence(()->drive.moveTo(new Pose2d(-38, 57, Math.toRadians(90)), 0));
    }

    protected void parkAtBlueObservationFromChamber(){
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
        drive.setSimpleMovePower(0.99);
        drive.moveTo(new Pose2d(-48, 57, Math.toRadians(90)), 100,()->Action.buildSequence(update));
    }

    protected void parkAtRedObservationFromChamber(){
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
        drive.setSimpleMovePower(0.99);
        drive.moveTo(new Pose2d(48, -57, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
    }


    protected Pose2d lastBlueBasketSample = new Pose2d(58, 47.5, Math.toRadians(-64));
    protected Pose2d lastRedSample = new Pose2d(-54,-45.0,Math.toRadians(125));
    protected void moveAndIntakeLastBasketSampleBlue(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(5));
        drive.setSimpleMovePower(0.3);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,50));
//        moveToGetLastYellowSample();
        drive.moveTo(lastBlueBasketSample, 200, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20,0.3),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);}));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);

    }
    protected Pose2d lastBlueChamberSample = new Pose2d(-58, 47, Math.toRadians(-64));

    protected void moveAndIntakeLastChamberSampleBlue(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(5));
        drive.setSimpleMovePower(0.3);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,50));
//        moveToGetLastYellowSample();
        drive.moveTo(lastBlueChamberSample, 200, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20,0.5),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);}));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }
    protected void moveAndIntakeLastBasketSampleRed(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
        drive.setSimpleMovePower(0.6);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,50));
//        moveToGetLastYellowSample();
        Action.actions.add(new SlideAction(upper, (int)0.9*SSValues.SLIDE_AUTO_INTAKE_LAST_RED,20,1));
        Action.buildSequence(()->{drive.moveTo(lastRedSample, 0); update.run();});
        drive.moveTo(lastRedSample, 600);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST_RED,20, 0.6));
        Action.buildSequence(update);
        sleep(60);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void moveAndIntakeLastHPSampleBlue(){
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,100));
//        moveToGetLastYellowSample();
        Action.actions.add(new SlideAction(upper, (int)0.8*SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20,1));
        Action.buildSequence(()->{drive.moveTo(new Pose2d(54, 47, Math.toRadians(-67)), 200); update.run();});
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20, 0.3));
        Action.buildSequence(update);
        sleep(70);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void resetAfterHighChamberAndMoveToIntakeFirst(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-28, 43, Math.toRadians(-140)), 300));
    }
    protected void resetAfterHighChamberAndMoveToIntakeSecond(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD, 100));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-40, 43, Math.toRadians(-140)), 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-40, 43, Math.toRadians(-140)), 200));
    }
    protected void prepareForPushTwoBlueSample(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 0));
        Action.buildSequence(update);
    }

    protected void intakeBlueSample(){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.7);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,0));
        Action.actions.add(new SlideAction(upper, (int)0.9*SSValues.SLIDE_AUTO_INTAKE_FIRST,20,1));
//        Action.buildSequence(()->drive.moveTo(new Pose2d(47.5+xOffset, 46, Math.toRadians(-90)), 350));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_FIRST,10, 0.4));
        Action.buildSequence(update);
        sleep(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void placeBlueSampleAtHP(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);
        drive.moveTo(new Pose2d(-40, 54, Math.toRadians(-90)), 0);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP,50));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN, 200));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-40, 54, Math.toRadians(-90)), 50));
        drive.moveTo(new Pose2d(-40, 60, Math.toRadians(-90)), 150);
    }

    protected void clawIntakePlace(){
        drive.moveTo(new Pose2d(-40, 60, Math.toRadians(-90)), 100);
    }
    protected void prepareForClawBlueSampleUp(double xOffset, double yOffset){
        Action.actions.add(new SlideAction(upper,SSValues.SLIDE_MIN,10));
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);
        drive.moveTo(new Pose2d(-57.5+xOffset, 46+yOffset, Math.toRadians(-90)), 200,()->Action.buildSequence(update));

    }
    protected void prepareForClawRedSampleUp(double xOffset, double yOffset){
        Action.actions.add(new SlideAction(upper,SSValues.SLIDE_MIN,10));
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.8);
        drive.moveTo(new Pose2d(57.5-xOffset, -49-yOffset, Math.toRadians(90)), 250,()->Action.buildSequence(update));
    }

    protected void expPrepareForClawBlueSampleUp(double xOffset, double yOffset, int waitTime){
        Action.actions.add(new SlideAction(upper,SSValues.SLIDE_MIN,10));
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.95);
        drive.moveTo(new Pose2d(-57.5+xOffset, 49+yOffset, Math.toRadians(-90)), 100+waitTime,()->Action.buildSequence(update));
    }

    protected void expPrepareForClawRedSampleUp(double xOffset, double yOffset, int extraTime){
        Action.actions.add(new SlideAction(upper,SSValues.SLIDE_MIN,10));
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(new Pose2d(57.5-xOffset, -49-yOffset, Math.toRadians(90)), 130+extraTime,()->Action.buildSequence(update));
    }

    protected void clawBlueSampleUp(double xOffset, double yOffset){
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 100));
//        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_MIN),new ArmAction(upper, SSValues.ARM_UP)));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-58.3+xOffset, 59.5+yOffset, Math.toRadians(-90)), 300,()->Action.buildSequence(update));
//        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
        Action.buildSequence(update);
        sleep(300);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 30));
        Action.buildSequence(update);
    }

    protected void clawRedSampleUp(double xOffset,double yOffset){
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 100));
//        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_MIN),new ArmAction(upper, SSValues.ARM_UP)));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(58.3-xOffset, -59.5-yOffset, Math.toRadians(90)), 300,()->Action.buildSequence(update));
//        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
        Action.buildSequence(update);
        sleep(300);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 30));
        Action.buildSequence(update);
    }

    protected void expClawBlueSampleUp(double xOffset,double yOffset){
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_MIN),new ArmAction(upper, SSValues.ARM_UP)));
        drive.moveTo(new Pose2d(-58.3+xOffset, 59.5+yOffset, Math.toRadians(-90)), 300,()->Action.buildSequence(update));
//        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
        Action.buildSequence(update);
        sleep(180);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 20));
        drive.moveTo(new Pose2d(-58.3+xOffset, 59.5+yOffset, Math.toRadians(-90)), 300,()->Action.buildSequence(update));
    }

    protected void VexpClawBlueSampleUp(double xOffset,double yOffset){
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_MIN),new ArmAction(upper, SSValues.ARM_UP)));
        drive.moveTo(new Pose2d(-58.3+xOffset, 59.5+yOffset, Math.toRadians(-90)), 300,()->Action.buildSequence(update));
//        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
        Action.buildSequence(update);
        sleep(180);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 20));
        drive.moveTo(new Pose2d(-58.3+xOffset, 59.5+yOffset, Math.toRadians(-90)), 300,()->Action.buildSequence(update));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER_OLD));
        drive.moveTo(new Pose2d(-58,57,Math.toRadians(180)),50, ()->{Action.buildSequence(update); update.run();});
    }

    protected void expClawRedSampleUp(double xOffset,double yOffset){
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 100));
//        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_MIN),new ArmAction(upper, SSValues.ARM_UP)));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(58.3-xOffset, -59.5-yOffset, Math.toRadians(90)), 300,()->Action.buildSequence(update));
//        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
        Action.buildSequence(update);
        sleep(180);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 20));
        Action.buildSequence(update);
    }



    ///////////////////////////////////////////////TESTS////////////////////////////////////////////

    protected void autoSplineTest(){
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(10, 10), 0)
                .build();

        drive.followTrajectory(traj);
    }

    protected void autoDriveTest(){
        drive.moveTo(new Pose2d(100,100,0),100);
    }

    protected void autoArmTest(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        upper.setArmTargetPosition(SSValues.ARM_UP);
//        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 10));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 10));
        Action.buildSequence(update);
    }

    protected void autoGrabFromSubmersibleTest(){
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.3),
                ()->upper.alphaAdjustedSampleColor().equals("blue") || upper.alphaAdjustedSampleColor().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->upper.setIntake(SSValues.CONTINUOUS_STOP)));
        Action.buildSequence(update);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        Action.buildSequence(update);
        delay(1000);
    }

    protected void autoResetArmTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN));
        Action.buildSequence(update);
    }

    protected void finishConditionActionTest(){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new FinishConditionActionGroup(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.3),
                ()->upper.alphaAdjustedSampleColor().equals("blue") || upper.alphaAdjustedSampleColor().equals("yellow"),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setWristPos(SSValues.WRIST_DEFAULT);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);}));
        Action.buildSequence(update);
        delay(1000);
    }

    public static  double testPIDx = 0, testPIDy = 0, testPIDheading = 90;
    public static  double targetPIDx = 0, targetPIDy = 40, targetPIDheading = 90;
    Pose2d[] poses = {new Pose2d(targetPIDx,targetPIDy,Math.toRadians(targetPIDheading)),new Pose2d(testPIDx,testPIDy,Math.toRadians(testPIDheading))};
    private Pose2d currentPose;
    private int poseCount = 0;
    protected void testAutoPID(){
        drive.setSimpleMovePower(0.5);
        if(!drive.isBusy()){
            if(poseCount < poses.length){
                currentPose = poses[poseCount];
                drive.moveTo(currentPose,200);
                poseCount++;
            }else{
                poseCount = 0;
            }
        }
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
//            idle();
            update.run();
//            upper.update();
        }
    }
}