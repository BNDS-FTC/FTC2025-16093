package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.uppersystems.Action;
import org.firstinspires.ftc.teamcode.uppersystems.ArmAction;
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
    private SuperStructure upper;
    private Runnable update;
    //TODO: Sketchy code
    private ArrayList<Action> actions = new ArrayList<Action>(6);

    Pose2d startPos;
    Pose2d boxPos;
    public static double box_x = 56.5, box_y = 53, box_heading = -45;
    Pose2d chamberPos;

    Pose2d intakeSamplePos_1;
    Pose2d intakeSamplePos_2;
    Pose2d intakeSamplePos_3;

    Pose2d pushSamplePos_1;
    Pose2d pushSamplePos_2;
    Pose2d pushSamplePos_3;

    Pose2d intakeSpecimenPos;


    protected void initHardware() throws InterruptedException{
        //TODO check if this start pose is correct (10% chance not correct)
        startPos = new Pose2d(15 * startSide ,62.3 * side_color,Math.toRadians(-90 * side_color));
        //TODO measure these because these are 100% not correct
        boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading * side_color));
        intakeSamplePos_1 = new Pose2d(57 * startSide, 48 * side_color, Math.toRadians(-90 * side_color));

        pushSamplePos_1 = new Pose2d(-40 * startSide, 40 * side_color, Math.toRadians(-90 * side_color));

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new BarkMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.update();
        drive.setSimpleMoveTolerance(2,Math.toRadians(10));

        telemetry.addLine("init: superstructure");
        telemetry.update();
        SuperStructure upper = new SuperStructure(
                this,
                () -> {
                });

        update = ()->{
            drive.update();
            telemetry.update();
        };

        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);



        telemetry.addLine("init: trajectory");
        telemetry.update();


    }

    protected void moveToHighChamber(){
        drive.setSimpleMoveTolerance(2,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(boxPos,1500);

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

    protected void intakeFloorSample(){
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}