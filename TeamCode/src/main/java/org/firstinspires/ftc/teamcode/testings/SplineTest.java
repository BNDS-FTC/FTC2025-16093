package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        drive.setUpdateRunnable(()->drive.update());
        drive.setPoseEstimate(new Pose2d(53,57,Math.toRadians(-130)));

        waitForStart();

        if (isStopRequested()) return;

//        drive.moveTo(new Pose2d(-58,57,Math.toRadians(180)),100);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(53,57,Math.toRadians(-130)))
                .splineToLinearHeading(new Pose2d(35,13,Math.toRadians(-180)), Math.toRadians(50))
                .build();
        drive.followTrajectory(traj);

        sleep(1000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(53, 57), Math.toRadians(-130))
                        .build()
        );
    }
}