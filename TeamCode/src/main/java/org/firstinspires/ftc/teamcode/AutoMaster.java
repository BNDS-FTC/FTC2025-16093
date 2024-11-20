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
import org.firstinspires.ftc.teamcode.uppersystems.SlideAction;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.uppersystems.WristAction;

import java.util.ArrayList;

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
    //TODO: Sketchy code
    private ArrayList<Action> actions = new ArrayList<Action>(6);

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


    protected void initHardware() throws InterruptedException{
        //TODO check if this start pose is correct (10% chance not correct)
        startPos = new Pose2d(-15  ,62.3 ,Math.toRadians(-90 ));
        //TODO measure these because these are 100% not correct
        boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading * startSide));
        chamberPos = new Pose2d(chamber_x * startSide, chamber_y * side_color, Math.toRadians(chamber_y * startSide));
        intakeSamplePos_1 = new Pose2d(57 * startSide, 48 * side_color, Math.toRadians(-90 * startSide));
        pushSamplePos_1 = new Pose2d(-40 * startSide, 40 * side_color, Math.toRadians(0 * startSide));
        hpZonePos = new Pose2d(hp_x * startSide, hp_y * side_color, Math.toRadians(90 * startSide));

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new BarkMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.update();
        drive.setSimpleMoveTolerance(2,Math.toRadians(10));

        update = ()->{
            drive.update();
            telemetry.update();
        };

        telemetry.addLine("init: superstructure");
        telemetry.update();
        SuperStructure upper = new SuperStructure(
                this,
                () -> {
                });

        update = ()->{
            drive.update();
            telemetry.update();
            upper.update();
        };

        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.9);
        upper.setArmByP(SSValues.ARM_DEFAULT, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_OPEN);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_OPEN);

        telemetry.addLine("init: trajectory");
        telemetry.update();


    }

    protected void reset(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        // Sequence actions based on last upper.getSequence()
        if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
            upper.setGrabPos(SSValues.GRAB_CLOSED);
            actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
            actions.add(new SlideAction(upper, SSValues.SLIDE_MIN));
        } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.HANG || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
            upper.setGrabPos(SSValues.GRAB_DEFAULT);
            actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
            actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
            actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT, 300));
        }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
            actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
            actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT,200));
        }
        upper.buildSequence(actions);
    }

    protected void moveToHighChamber(){
//        upper.switchSequence(SuperStructure.Sequences.RUN);
//        drive.setSimpleMoveTolerance(2,Math.toRadians(5));
//        drive.setSimpleMovePower(0.9);
//        drive.moveTo(new Pose2d(-40,50,Math.toRadians(90)),1500);

    }

    protected void simplePushSample1Blue(){
        drive.setSimpleMoveTolerance(2, Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(new Pose2d(-38, 50, Math.toRadians(0)), 1500);
        drive.moveTo(new Pose2d(-38, 13, Math.toRadians(0)), 1500);
        drive.moveTo(new Pose2d(-47, 13, Math.toRadians(0)), 1500);
        drive.moveTo(new Pose2d(-47, 58, Math.toRadians(0)), 1500);
        drive.moveTo(new Pose2d(-57, 13, Math.toRadians(0)), 1500);
        drive.moveTo(new Pose2d(-57, 58, Math.toRadians(0)), 1500);
    }

    protected void highChamberPlace(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
//        actions.add(new ArmAction(upper, SSValues.ARM_HIGH_BASKET,200));
        actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM, 20));
        drive.moveTo(chamberPos,200);
        actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE, 20));
        actions.add(new ClawAction(upper, SSValues.CLAW_RIGHT_OPEN,SSValues.CLAW_LEFT_OPEN));
    }

    protected void intakeFloorSample(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
    }

    protected void moveToPushSample1(){
        drive.setSimpleMoveTolerance(2,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(pushSamplePos_1,1500);
    }
    protected void pushSample(){
        drive.setSimpleMoveTolerance(2,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(sampleToHPZonePos,1500);
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}