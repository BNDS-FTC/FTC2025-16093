package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@Config
public abstract class AutoMaster extends LinearOpMode {
    private ElapsedTime runtime;
    private NewMecanumDrive drive;
    private SuperStructure upper;
    private Runnable update;

    private Pose2d startPos = new Pose2d();

    public void initHardware() throws InterruptedException{
        runtime = new ElapsedTime();
        runtime.reset();
        telemetry.addLine("init: drive");

        drive = new NewMecanumDrive();
        upper = new SuperStructure(
                this,
                () -> {
                    logic_period();
                    drive_period();
                });

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(startPos);
        drive.update();
        drive.setSimpleMoveTolerance(5,5,Math.toRadians(10));

        update = () -> {
            drive.update();
            upper.update();
        };
        upper.setUpdateRunnable(update);
        drive.setUpdateRunnable(update);

        while (opModeInInit()){
            drive.update();
        }

    }

    private void drive_period() {
        //add something later
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }


}
