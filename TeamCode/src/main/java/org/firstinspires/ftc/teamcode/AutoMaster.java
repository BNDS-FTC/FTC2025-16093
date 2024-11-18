package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

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
        // TODO: must make sure that these poses are correct
        startPos = new Pose2d(15 * startSide ,62.3 * side_color,Math.toRadians(-90 * side_color));
        boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading * side_color));

        //TODO these aren't tested
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
            upper.update();
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