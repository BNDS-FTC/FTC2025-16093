package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

@TeleOp(name="Semicircle Test", group="testing")
@Config
@Disabled
public class SemicircleTest extends LinearOpMode {
    public static double speedDiff = 0.5;
    NewMecanumDrive drive;

    @Override
    public void runOpMode() {
        waitForStart();
        drive = new NewMecanumDrive(hardwareMap);
        drive.setUpdateRunnable(()->{telemetryStuff();});
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while (opModeIsActive()) {
            drive.update();
            telemetryStuff();

            if(gamepad1.y){
                drive.stitchSemicircleTo(drive.getPoseEstimate(), new Pose2d(10,0,Math.toRadians(-90)), 200);
            }

            if(gamepad1.a){
                drive.resetHeading();
                drive.resetOdo();
            }
        }
    }

    public void telemetryStuff(){
        telemetry.addData("heading",Math.toDegrees(drive.getHeading()));
        telemetry.addData("lf",drive.leftFront.getPower());
        telemetry.addData("rf",drive.rightFront.getPower());
        telemetry.addData("lb",drive.leftRear.getPower());
        telemetry.addData("rb",drive.rightRear.getPower());
        telemetry.addData("stop semicircling", drive.stopSemicircling);
        telemetry.update();
    }
}
