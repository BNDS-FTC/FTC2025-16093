package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.AltMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.Action;
import org.firstinspires.ftc.teamcode.uppersystems.ArmAction;
import org.firstinspires.ftc.teamcode.uppersystems.ClawAction;
import org.firstinspires.ftc.teamcode.uppersystems.GrabAction;
import org.firstinspires.ftc.teamcode.uppersystems.ParallelActionGroup;
import org.firstinspires.ftc.teamcode.uppersystems.SlideAction;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.uppersystems.WristAction;

@Config
public abstract class AutoMaster extends LinearOpMode {

    private AltMecanumDrive drive;
    protected SuperStructure upper;
    protected Runnable update;

    Pose2d startPos;


    protected void initHardware(Pose2d start) throws InterruptedException{
        startPos = start;

        telemetry.addLine("init: drive");
        telemetry.update();
//        drive = new NewMecanumDrive(hardwareMap);
        drive = new AltMecanumDrive(hardwareMap);
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
            telemetry.addData("In Distress?", drive.simpleMoveInDistress);
            telemetry.addData("Current Pose", drive.getCurrentPoseAsString());
            telemetry.addData("Target Pose",(drive.simpleMoveIsActivate)? drive.getSimpleMovePosition().toString() : "SimpleMove not activated");
            telemetry.addData("Arm Position: ", upper.getArmPosition());
            telemetry.addLine(Action.showCurrentAction());
            telemetry.update();
            upper.update();
            if (Action.actions.isEmpty()) {
                if (resetArm.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN) {
                    upper.resetArmEncoder();
                    upper.resetSlideEncoder();
                }
            }
            if(drive.simpleMoveInDistress){
                prepareForTeleOpWithoutMoving();
                requestOpModeStop();
            }
        };

        drive.setUpdateRunnable(update);

        upper.resetSlide();
        upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);//Maybe we should test this!
        upper.setArmByP(SSValues.ARM_DOWN, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);

        telemetry.addLine("init: trajectory");
        telemetry.update();


    }

    ///////////////////////////////////BLUE BASKET///////////////////////////////////////////////
    protected void reset(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        // Sequence actions based on last sequence
        if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
            upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
//            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
        } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.ASCENT || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
            upper.setGrabPos(SSValues.GRAB_DEFAULT);
            Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
            Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT));
        }
        Action.buildSequence(update);
    }
    protected Pose2d yellowPose;
    protected void resetAfterBlueBasketAndMoveToIntake(double xOffset, double headingOffset){
        yellowPose = new Pose2d(47.5+xOffset, 46, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.75);
        drive.moveTo(new Pose2d(50, 50, Math.toRadians(-135)), 0);
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

    protected void expResetAfterBlueBasketAndMoveToIntake(double xOffset, double headingOffset){
        yellowPose = new Pose2d(47.5+xOffset, 46, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.85);
        drive.moveTo(new Pose2d(50, 50, Math.toRadians(-135)), 0);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 100));
        Action.buildSequence(update);
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
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER,800));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(-10+xOffset, 41.5, Math.toRadians(91)), 150,()->Action.buildSequence(update));
//        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
//        drive.setSimpleMovePower(0.7);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(-10+xOffset, 35, Math.toRadians(90)), 100,()->Action.buildSequence(update));
    }

    protected void moveToRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER,800));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(10-xOffset, -42, Math.toRadians(-89)), 100,()->Action.buildSequence(update));
//        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
//        drive.setSimpleMovePower(0.7);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(10-xOffset, -35, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
    }

    protected void expMoveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.95);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
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
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER,1400));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(10-xOffset, -42, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
        drive.setSimpleMoveTolerance(1.2, 1, Math.toRadians(5));
        drive.setSimpleMovePower(0.8);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(10-xOffset, -35, Math.toRadians(-90)), 100,()->Action.buildSequence(update));
    }

    protected void firstMoveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.6);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 600));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
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
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
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
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(-10+xOffset, 36, Math.toRadians(90)),0,()->Action.buildSequence(update));
    }

    protected void expFirstMoveToRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(1.5,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.6);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 700));
        Action.buildSequence(update);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM_AUTO,40));
        drive.moveTo(new Pose2d(10-xOffset, -36, Math.toRadians(-90)),0,()->Action.buildSequence(update));
    }

    protected void highChamberPlace(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,100));
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_OPEN, SSValues.CLAW_RIGHT_OPEN,70));
        Action.buildSequence(update);
    }


    ///////////////////////////////////BLUE//////////////////////////////////////////

//    protected void moveToBlueChamberAim(){
//        drive.setSimpleMoveTolerance(2,0.7, Math.toRadians(5));
//        drive.setSimpleMovePower(0.9);
//        drive.moveTo(new Pose2d(0, 43, Math.toRadians(90)), 500);
//        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
//        Action.buildSequence(update);
//    }




    protected Pose2d blueBasket = new Pose2d(52.3, 55, Math.toRadians(-135));
    protected Pose2d redBasket = new Pose2d(-52.3, -55.3, Math.toRadians(45));
    protected void firstPutBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,0));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(53, 54, Math.toRadians(-135)), 800,()->Action.buildSequence(update));
//        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        delay(200);
    }

    protected void firstPutRedBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
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
        drive.setSimpleMovePower(0.6);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 500));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,0));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(53, 54, Math.toRadians(-135)), 500,()->Action.buildSequence(update));
//        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        delay(200);
    }

    protected void putBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.5);
//        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600);
        Action.actions.add(new GrabAction(upper, SSValues.AUTO_GRAB_CLOSED, 20));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 400));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,600));
        drive.moveTo(blueBasket, 400,()->Action.buildSequence(update));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        sleep(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
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
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        drive.moveTo(new Pose2d(-36, 40, Math.toRadians(90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveTo(new Pose2d(-36, 15, Math.toRadians(-135)), 0,()->Action.buildSequence(update));
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
        drive.setSimpleMovePower(1);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        drive.moveTo(new Pose2d(-36, 40, Math.toRadians(90)), 0,()->Action.buildSequence(update));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        drive.moveToWithDe(new Pose2d(-36, 15, Math.toRadians(90)), 0,()->Action.buildSequence(update),true);
        drive.moveWithDrift(new Pose2d(-47, 52, Math.toRadians(90)),new Pose2d(-47, 15, Math.toRadians(90)),new Pose2d(-55.5, 15, Math.toRadians(90)),new Pose2d(-56, 52, Math.toRadians(90)));
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

    protected void moveToGetLastYellowSample(){
        drive.moveTo(new Pose2d(54, 47, Math.toRadians(-65)), 200);
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


    protected Pose2d lastBlueSample = new Pose2d(54, 46, Math.toRadians(-60));
    protected Pose2d lastRedSample = new Pose2d(-54,-45.0,Math.toRadians(125));
    protected void moveAndIntakeLastBasketSampleBlue(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
        drive.setSimpleMovePower(0.6);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,50));
//        moveToGetLastYellowSample();
        Action.actions.add(new SlideAction(upper, (int)0.9*SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20,1));
        Action.buildSequence(()->{drive.moveTo(lastBlueSample, 0); update.run();});
        drive.moveTo(lastBlueSample, 600);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST_BLUE,20, 0.6));
        Action.buildSequence(update);
        sleep(60);
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
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER, 100));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-28, 43, Math.toRadians(-140)), 300));
    }
    protected void resetAfterHighChamberAndMoveToIntakeSecond(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER, 100));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 500));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-40, 43, Math.toRadians(-140)), 0));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN,200));
        Action.buildSequence(()->drive.moveTo(new Pose2d(-40, 43, Math.toRadians(-140)), 200));
    }
    protected void prepareForPushTwoBlueSample(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER, 100));
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
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
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
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
        Action.buildSequence(update);
        sleep(300);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 30));
        Action.buildSequence(update);
    }

    protected void expClawBlueSampleUp(double xOffset,double yOffset){
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_MIN),new ArmAction(upper, SSValues.ARM_UP)));
        drive.moveTo(new Pose2d(-58.3+xOffset, 59.5+yOffset, Math.toRadians(-90)), 300,()->Action.buildSequence(update));
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
        Action.buildSequence(update);
        sleep(180);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 20));
        drive.moveTo(new Pose2d(-58.3+xOffset, 59.5+yOffset, Math.toRadians(-90)), 300,()->Action.buildSequence(update));
    }

    protected void expClawRedSampleUp(double xOffset,double yOffset){
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
//        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 100));
//        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new ParallelActionGroup(new SlideAction(upper, SSValues.SLIDE_MIN),new ArmAction(upper, SSValues.ARM_UP)));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(58.3-xOffset, -59.5-yOffset, Math.toRadians(90)), 300,()->Action.buildSequence(update));
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE));
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

    protected void autoUpperTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX));
        Action.buildSequence(update);
        delay(15000);
    }

    protected void autoResetArmTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN));
        Action.buildSequence(update);
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