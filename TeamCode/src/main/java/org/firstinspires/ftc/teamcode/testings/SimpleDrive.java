package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

@TeleOp(name="simple drive")
public class SimpleDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);;
        while (opModeIsActive()) {
            telemetry.addData("heading",Math.toDegrees(drive.getHeading()));
            telemetry.update();
            drive.update();
            if(gamepad1.a){
                drive.resetHeading();
                drive.resetOdo();
            }
            drive.setGlobalPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, null);
        }
    }
}
