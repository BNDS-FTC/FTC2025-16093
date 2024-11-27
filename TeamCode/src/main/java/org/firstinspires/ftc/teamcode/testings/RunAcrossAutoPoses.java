package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

@TeleOp (group = "Testing")
@Config
public class RunAcrossAutoPoses extends LinearOpMode {
    //    NewMecanumDrive drive = new NewMecanumDrive();
    BarkMecanumDrive drive;

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static  double x = 15, y = 62.3, heading = 270;
    Pose2d[] poses = {new Pose2d(40,62.3,Math.toRadians(270)),
            new Pose2d(40,-62.3,Math.toRadians(90)),
            new Pose2d(15,-62.3,Math.toRadians(90)),
            new Pose2d(-15,-62.3,Math.toRadians(90)),
            new Pose2d(-40,-62.3,Math.toRadians(90)),
            new Pose2d(-40,62.3,Math.toRadians(270)),
            new Pose2d(15,62.3,Math.toRadians(270)),
            new Pose2d(15,62.3,Math.toRadians(270))
    };
    private static Pose2d startPos;
    int count;
    @Override
    public void runOpMode(){
        XCYBoolean testMove = new XCYBoolean(()-> gamepad1.b);
        drive = new BarkMecanumDrive(hardwareMap);

        Runnable update = ()->{drive.update();XCYBoolean.bulkRead();};
        drive.setUpdateRunnable(update);

        startPos = new Pose2d(x,y,Math.toRadians(heading));
        drive.setPoseEstimate(startPos);
        drive.update();
        telemetry.addData("Pos Estimate: ",drive.getPoseEstimate());
        telemetry.update();
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        count = 0;
        XCYBoolean a = new XCYBoolean(() -> gamepad1.a);


        waitForStart();

        while (opModeIsActive()){
            double standard_xPos = drive.getPoseEstimate().getX();
            double standard_yPos = drive.getPoseEstimate().getY();

            if(a.toTrue()){
                if(count < poses.length){
                    drive.moveTo(poses[count],3000);
                    count++;
                }else{
                    count = 0;
                }
            }
            if(gamepad1.b){
                drive.stopTrajectory();
                drive.setMotorPowers(
                        0,0,0,0
                );
            }
            Pose2d error = drive.getLastError();
            telemetry.addData("Current X Position (in): ", "%.3f", standard_xPos);
            telemetry.addData("Current Y Position (in): ", "%.3f", standard_yPos);
            telemetry.addData("Current Heading: ", drive.getPoseEstimate().getHeading());

            telemetry.addData("Error: ", error);
            telemetry.update();
            update.run();
        }
    }
}
