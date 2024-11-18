package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@Autonomous
public class AutoRedBasket extends LinearOpMode {
    private ElapsedTime runtime;
    private NewMecanumDrive drive;
    private SuperStructure upper;
    private Runnable update;

    Pose2d startPos = new Pose2d(-12,-54, Math.toRadians(90));
    Pose2d LowBox = new Pose2d(-52,-45,Math.toRadians(45));

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
        drive = new NewMecanumDrive();

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(startPos);
        drive.update();
        drive.getLocalizer().setPoseEstimate(startPos);
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
        drive.moveTo(LowBox,500);
    }

    private void drive_period() {
        //add something here
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }
}
