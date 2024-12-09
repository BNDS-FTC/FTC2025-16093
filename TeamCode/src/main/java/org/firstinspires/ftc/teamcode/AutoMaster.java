package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
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

    private BarkMecanumDrive drive;
    protected SuperStructure upper;
    private Runnable update;

    Pose2d startPos;
    Pose2d boxPos;
    public static double box_x = 56.5, box_y = 53, box_heading = -45;
    Pose2d chamberPos;
    public static double chamber_x = 0, chamber_y = 45, chamber_heading = 90;
    Pose2d hpZonePos;
    public static double hp_x = 0, hp_y = 45, hp_heading = 90;
    Pose2d sampleToHPZonePos;
    public static double pushToHp_x = 40, pushToHp_y = 55, pushToHp_heading = 0;

    Pose2d intakeSamplePos_1;
    Pose2d intakeSamplePos_2;
    Pose2d intakeSamplePos_3;

    Pose2d pushSamplePos_1;
    Pose2d pushSamplePos_2;
    Pose2d pushSamplePos_3;

    Pose2d intakeSpecimenPos;


    protected void initHardware(Pose2d start) throws InterruptedException{
        //TODO check if this start pose is correct (10% chance not correct)
//        startPos = new Pose2d(-15  ,62.3 ,Math.toRadians(90));
        startPos = start;
        //TODO measure these because these are 100% not correct
        boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading * startSide));
        chamberPos = new Pose2d(0 * startSide, chamber_y * side_color, Math.toRadians(chamber_y * startSide));
        intakeSamplePos_1 = new Pose2d(57 * startSide, 48 * side_color, Math.toRadians(-90 * startSide));
        pushSamplePos_1 = new Pose2d(-40 * startSide, 40 * side_color, Math.toRadians(0 * startSide));
        hpZonePos = new Pose2d(hp_x * startSide, hp_y * side_color, Math.toRadians(90 * startSide));

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new BarkMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.update();
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(10));

        update = ()->{
            drive.update();
            telemetry.update();
        };

        telemetry.addLine("init: superstructure");
        telemetry.update();
        upper = new SuperStructure(
                this,
                () -> {
                }, SSValues.AUTO_ARM_OFFSET);

        update = ()->{
            drive.update();
            telemetry.update();
            upper.update();
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

    protected void resetAfterBlueBasketAndMoveToIntake(double xOffset){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.moveTo(new Pose2d(50, 50, Math.toRadians(-135)), 0);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES, 50));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DOWN, 300));
        Action.buildSequence(()->{});
        drive.moveTo(new Pose2d(47.5+xOffset, 46, Math.toRadians(-90)), 350);
    }


    protected void prepareForTeleOp(){
        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_CLOSED);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);//Maybe we should test this!
        upper.setArmByP(SSValues.ARM_DOWN -SSValues.AUTO_ARM_OFFSET, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);
        sleep(500);
    }

    protected void highChamberAim(){
        drive.setSimpleMoveTolerance(2,0.7, Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM,50));
        Action.buildSequence(()->drive.moveTo(new Pose2d(0, 43, Math.toRadians(90)), 500));
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

    protected void moveToBlueChamberPlace(){
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(new Pose2d(0, 33, Math.toRadians(90)), 100);
    }

    protected void firstPutBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 400);
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

    protected void putBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
//        drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600);
        upper.setGrabPos(SSValues.GRAB_CLOSED);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 200));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP, 50));
        Action.buildSequence(update);
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MAX, 50));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_RELEASE,1000));
        Action.buildSequence(()->drive.moveTo(new Pose2d(53.5, 51.5, Math.toRadians(-135)), 600));
        sleep(400);
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN));
        Action.actions.add(new WaitAction(200));
        Action.buildSequence(update);
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
    protected void getYellowSamples(double xOffset){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        drive.setSimpleMoveTolerance(0.7,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.75);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.startIntake();
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,100));
        Action.actions.add(new SlideAction(upper, (int)0.8*SSValues.SLIDE_AUTO_INTAKE_YELLOW,20,1));
//        Action.buildSequence(()->drive.moveTo(new Pose2d(47.5+xOffset, 46, Math.toRadians(-90)), 350));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_YELLOW,10, 0.3));
        Action.buildSequence(update);
        sleep(100);
        upper.stopIntake();
    }

    protected void moveToGetLastYellowSample(){
        drive.moveTo(new Pose2d(54, 47, Math.toRadians(-65)), 200);
    }

    protected void parkAtObservationFromBasket(){
        drive.moveTo(new Pose2d(-34, 61, Math.toRadians(-90)), 0);
    }

    protected void moveAndIntakeLastSample(){
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,100));
//        moveToGetLastYellowSample();
        Action.actions.add(new SlideAction(upper, (int)0.8*SSValues.SLIDE_AUTO_INTAKE_LAST,20,1));
        Action.buildSequence(()->drive.moveTo(new Pose2d(54, 47, Math.toRadians(-65)), 200));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.startIntake();
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST,20, 0.3));
        Action.buildSequence(update);
        sleep(100);
        upper.stopIntake();
    }

    protected void placeLastBlueSampleAtHP(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(new Pose2d(-40, 54, Math.toRadians(-90)), 200);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP,50));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN, 200));
        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-40, 59, Math.toRadians(-90)), 100);
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