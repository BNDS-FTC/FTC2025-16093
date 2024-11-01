package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.AltMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.GobildaPinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@Autonomous
public class BadAutoTest extends LinearOpMode {
    private ElapsedTime runtime;
    private AltMecanumDrive drive;
    private SuperStructure upper;
    private Runnable update;
    private GobildaPinpointLocalizer localizer;

    Pose2D startPos = new Pose2D(DistanceUnit.MM,-12,-54, AngleUnit.DEGREES,90);
    Pose2D lowBox = new Pose2D(DistanceUnit.MM,-52,-45, AngleUnit.DEGREES,45);

    @Override
    public void runOpMode(){
        runtime = new ElapsedTime();
        runtime.reset();
        telemetry.addLine("init: drive");
        telemetry.update();

        upper = new SuperStructure(
                this,
                () -> {
                    logic_period();
                    drive_period();
                });
        drive = new AltMecanumDrive();
        drive.setUp(hardwareMap);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(convertPose2d(startPos));
        drive.update();
        drive.getLocalizer().setPoseEstimate(convertPose2d(startPos));
        drive.update();
        drive.setSimpleMoveTolerance(5,5, Math.toRadians(3));

        update = () -> {
            drive.update();
            upper.update();
        };
        upper.setUpdateRunnable(update);
        drive.setUpdateRunnable(update);

        while(opModeInInit()){
            drive.setSimpleMovePower(0);
        }

        drive.setSimpleMovePower(0.95);
        drive.moveTo(convertPose2d(lowBox),500);
    }

    private void drive_period() {
        //add something here
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }

    public Pose2d convertPose2d(Pose2D p){
        return new Pose2d(p.getX(DistanceUnit.MM),p.getY(DistanceUnit.MM),p.getHeading(AngleUnit.RADIANS));
    }
}
