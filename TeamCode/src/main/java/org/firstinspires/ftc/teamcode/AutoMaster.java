package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.Action;
import org.firstinspires.ftc.teamcode.uppersystems.ArmAction;
import org.firstinspires.ftc.teamcode.uppersystems.ClawAction;
import org.firstinspires.ftc.teamcode.uppersystems.GrabAction;
import org.firstinspires.ftc.teamcode.uppersystems.SlideAction;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.uppersystems.WaitAction;
import org.firstinspires.ftc.teamcode.uppersystems.WristAction;

@Config
public abstract class AutoMaster extends LinearOpMode {

    public static final int POSITIVE = 1;
    public static final int NEGATIVE = -1;
    public static final int RED = -1;
    public static final int BLUE = 1;

    protected int startSide;
    protected int side_color;

    private NewMecanumDrive drive;
    protected SuperStructure upper;
    protected Runnable update;

    Pose2d startPos;
    Pose2d boxPos;
    public static double box_x = 56.5, box_y = 53, box_heading = -45;
    Pose2d chamberPos;
    public static double chamber_x = 0, chamber_y = 45, chamber_heading = 90;
    Pose2d hpZonePos;
    public static double hp_x = 0, hp_y = 45, hp_heading = 90;
    Pose2d sampleToHPZonePos;
    public static double pushToHp_x = 40, pushToHp_y = 55, pushToHp_heading = 0;



    protected void initHardware(Pose2d start) throws InterruptedException{
        //TODO check if this start pose is correct (10% chance not correct)
//        startPos = new Pose2d(-15  ,62.3 ,Math.toRadians(90));
        startPos = start;

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new NewMecanumDrive(hardwareMap);
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
            telemetry.update();
            upper.update();
            if (Action.actions.isEmpty()) {
                if (resetArm.toTrue() && upper.getSequence() == SuperStructure.Sequences.RUN) {
                    upper.resetArmEncoder();
                    upper.resetSlideEncoder();
                }
            }
        };

        drive.setUpdateRunnable(update);

        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_CLOSED);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);//Maybe we should test this!
        upper.setArmByP(SSValues.ARM_DOWN, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);

        telemetry.addLine("init: trajectory");
        telemetry.update();


    }

    ///////////////////////////////////SUPERSTRUCTURE///////////////////////////////////////////////
    protected void reset(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        // Sequence actions based on last sequence
        if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
            upper.setGrabPos(SSValues.GRAB_CLOSED);
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
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        Action.buildSequence(()->{drive.moveTo(yellowPose, 200);update.run();});
        drive.moveTo(yellowPose, 300);
    }

    protected void prepareForTeleOp(){
        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_CLOSED);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);//Maybe we should test this!
        upper.setArmByP(SSValues.ARM_DOWN -SSValues.AUTO_ARM_OFFSET, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
        sleep(500);
    }

    protected void blueHighChamberAim(double xOffset){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.buildSequence(update);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.buildSequence(()->drive.moveTo(new Pose2d(0+xOffset, 43, Math.toRadians(90)), 0));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM,40));
        Action.buildSequence(update);
        drive.moveTo(new Pose2d(0+xOffset, 43, Math.toRadians(90)), 100);
    }

    protected void highChamberPlace(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,70));
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

    protected void moveToBlueChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(2, 5, Math.toRadians(5));
        drive.setSimpleMovePower(0.5);
        drive.moveTo(new Pose2d(0+xOffset, 32, Math.toRadians(90)), 100);
    }

    protected void firstPutBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.6);
        drive.moveTo(new Pose2d(53, 54, Math.toRadians(-135)), 500);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 300));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,800));
        Action.buildSequence(update);
        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.actions.add(new WaitAction(200));
        Action.buildSequence(update);
    }

    protected Pose2d blueBasket = new Pose2d(53, 55, Math.toRadians(-135));
    protected void putBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);
//        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_CLOSED, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 400));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,800));
        Action.buildSequence(()->{drive.moveTo(blueBasket, 0); update.run();});
        drive.moveTo(blueBasket, 300);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.buildSequence(update);
        sleep(200);
    }

    protected void pushTwoBlueSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(9,9, Math.toRadians(3));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(new Pose2d(-35, 36, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-37.5, 15, Math.toRadians(90)), 100);
        drive.moveTo(new Pose2d(-47.5, 10, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-47.5, 54, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-47.5, 15, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-57.5, 15, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-57.5, 54, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-55.5, 48.5, Math.toRadians(-125)), 200);
    }
    protected void getYellowSamples(){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.7);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.startIntake();
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,0));
        Action.actions.add(new SlideAction(upper, (int)0.9*SSValues.SLIDE_AUTO_INTAKE_YELLOW,20,1));
//        Action.buildSequence(()->drive.moveTo(new Pose2d(47.5+xOffset, 46, Math.toRadians(-90)), 350));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_YELLOW,10, 0.4));
        Action.buildSequence(update);
        sleep(100);
        upper.stopIntake();
    }

    protected void moveToGetLastYellowSample(){
        drive.moveTo(new Pose2d(54, 47, Math.toRadians(-65)), 200);
    }

    protected void parkAtObservationFromBasket(){
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

    protected Pose2d lastBlueSample = new Pose2d(54, 46, Math.toRadians(-60));
    protected void moveAndIntakeLastBasketSampleBlue(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
        drive.setSimpleMovePower(0.6);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,50));
//        moveToGetLastYellowSample();
        Action.actions.add(new SlideAction(upper, (int)0.9*SSValues.SLIDE_AUTO_INTAKE_LAST,20,1));
        Action.buildSequence(()->{drive.moveTo(lastBlueSample, 0); update.run();});
        drive.moveTo(lastBlueSample, 400);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.startIntake();
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST,20, 0.6));
        Action.buildSequence(update);
        sleep(100);
        upper.stopIntake();
    }

    protected void moveAndIntakeLastHPSampleBlue(){
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,100));
//        moveToGetLastYellowSample();
        Action.actions.add(new SlideAction(upper, (int)0.8*SSValues.SLIDE_AUTO_INTAKE_LAST,20,1));
        Action.buildSequence(()->{drive.moveTo(new Pose2d(54, 47, Math.toRadians(-67)), 200); update.run();});
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.startIntake();
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST,20, 0.3));
        Action.buildSequence(update);
        sleep(70);
        upper.stopIntake();
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

    protected void intakeBlueSample(){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.7);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.startIntake();
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,0));
        Action.actions.add(new SlideAction(upper, (int)0.9*SSValues.SLIDE_AUTO_INTAKE_FIRST,20,1));
//        Action.buildSequence(()->drive.moveTo(new Pose2d(47.5+xOffset, 46, Math.toRadians(-90)), 350));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_FIRST,10, 0.4));
        Action.buildSequence(update);
        sleep(100);
        upper.stopIntake();
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

    protected void clawBlueSampleUp(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new WaitAction(300));
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE, 200));
        Action.buildSequence(update);
        sleep(300);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 30));
        Action.buildSequence(update);
    }



    ///////////////////////////////////////////////TESTS////////////////////////////////////////////

    protected void autoUpperTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 5000));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        delay(5000);
        Action.buildSequence(update);
    }

    protected void autoResetArmTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN));
        Action.buildSequence(update);
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}