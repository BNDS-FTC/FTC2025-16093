package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

@TeleOp (group = "Testing")
@Config
public class TestTurnSpeed extends LinearOpMode {
    //    NewMecanumDrive drive = new NewMecanumDrive();
    NewMecanumDrive drive;

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static  double x = 0, y = 0, heading = 0;
    public static double targetX = 0, targetY = 0, targetHeading = 0;

    Pose2d[] allPoses = {new Pose2d(0,0,0), new Pose2d(1,1,Math.toRadians(15)), new Pose2d(0,0,Math.toRadians(30)), new Pose2d(0,0,Math.toRadians(45)), new Pose2d(0,0,Math.toRadians(90))};
    Pose2d[] poses = {new Pose2d(targetX,targetY,Math.toRadians(targetHeading)),new Pose2d(x,y,Math.toRadians(heading))};
    private static Pose2d startPos;
    Pose2d currentPose;
    int count;
    @Override
    public void runOpMode(){
        XCYBoolean testMove = new XCYBoolean(()-> gamepad1.b);
        drive = new NewMecanumDrive(hardwareMap);

        Runnable update = ()->{drive.update();XCYBoolean.bulkRead();};
        drive.setUpdateRunnable(update);

        startPos = new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPos);
        drive.update();
        telemetry.addData("Pos Estimate: ",drive.getPoseEstimate());
        telemetry.update();
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(10));
        double angle = 0;
        double x = 0;
        double y = 0;
        XCYBoolean a = new XCYBoolean(() -> gamepad1.a);


        waitForStart();

        while (opModeIsActive()){
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.setSimpleMovePower(1);
            drive.moveWithoutStopping(new Pose2d(x,y,Math.toRadians(angle)),0);
            angle += 10;
            x++;
            y++;


//            if(!drive.isBusy()){
//
//                if(count < poses.length){
//                    drive.moveTo(new Pose2d(0,0,angle),0);
//                    angle += 5;
////                    currentPose = poses[count];
////                    drive.moveTo(currentPose,200);
////                    count++;
//                }
////                else{
////                    count = 0;
////                }
//            }

            if(gamepad1.b){
                drive.stopTrajectory();
                drive.setMotorPowers(
                        0,0,0,0
                );
            }

            Pose2d error = drive.getLastError();

//            telemetry.addData("Current X Position (in): ", "%.3f", standard_xPos);
//            telemetry.addData("Current Y Position (in): ", "%.3f", standard_yPos);
//            telemetry.addData("Current Heading: ", drive.getPoseEstimate().getHeading());
//            telemetry.addData("Error: ", error);
//            telemetry.update();

//            telemetry_M.addData("X Error",currentPose.getX() - drive.getPoseEstimate().getX());
//            telemetry_M.addData("Y Error",currentPose.getY() - drive.getPoseEstimate().getY());
//            telemetry_M.addData("Heading Error", currentPose.getHeading() - drive.getPoseEstimate().getHeading());
//            telemetry_M.update();
        }
    }
}
