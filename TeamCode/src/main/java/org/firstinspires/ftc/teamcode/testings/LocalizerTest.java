package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GobildaPinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardLocalizer;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

@TeleOp (group = "Testing")
public class LocalizerTest extends LinearOpMode {
    GobildaPinpointLocalizer localizer;
    NewMecanumDrive drive = new NewMecanumDrive();
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode(){
        localizer = new GobildaPinpointLocalizer(hardwareMap);
        StandardLocalizer standardLocalizer = new StandardLocalizer(hardwareMap);
        XCYBoolean testMove = new XCYBoolean(()->gamepad1.b);

        drive.setUp(hardwareMap);
        drive.setPoseEstimate(new Pose2d(24,-24,Math.toRadians(90)));
        drive.update();
        drive.getLocalizer().setPoseEstimate(new Pose2d(24,-24,Math.toRadians(90)));
        drive.update();
        telemetry.addData("Pos Estimate: ",drive.getPoseEstimate());
        telemetry.update();

        Runnable update = ()->{drive.update();XCYBoolean.bulkRead();};

        waitForStart();

        while (opModeIsActive()){
            double xPosition = localizer.getWheelPositions().get(0);
            double yPosition = localizer.getWheelPositions().get(1);

            telemetry.addData("X Position (in): ", "%.3f", xPosition);
            telemetry.addData("Y Position (in): ", "%.3f", yPosition);
            //telemetry.addData("(x , y): ","%.3lf", localizer.getWheelPositions());
            telemetry.addData("Heading: ",localizer.getHeading());

            double standard_xPos = drive.getPoseEstimate().getX();
            double standard_yPos = drive.getPoseEstimate().getY();

            if(testMove.toTrue()){
                drive.moveTo(new Pose2d(24,0,Math.toRadians(0)),100);
            }
            if (testMove.toFalse()){
                drive.stopTrajectory();
                drive.setMotorPowers(
                        0,0,0,0
                );
            }

            telemetry.addData("Standard X Position (in): ", "%.3f", standard_xPos);
            telemetry.addData("Standard Y Position (in): ", "%.3f", standard_yPos);
            //telemetry.addData("(x , y): ","%.3lf", localizer.getWheelPositions());
            telemetry.addData("Standard Heading: ", drive.getPoseEstimate().getHeading());
            telemetry.update();

            update.run();
        }
    }
}
