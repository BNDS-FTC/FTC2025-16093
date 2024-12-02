package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.drive_basis.*;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.BarkMecanumDrive;

@TeleOp
public class Command_base_opMode extends LinearOpMode {

    static final boolean FIELD_CENTRIC = false;

    @Override
    public void runOpMode() throws InterruptedException {


        MecanumDrive drive = new MecanumDrive(
                //new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435)
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftBack"),
                new Motor(hardwareMap, "rightBack")
        );



        waitForStart();

        while (!isStopRequested()) {

            if (!FIELD_CENTRIC) {
                drive.driveRobotCentric(
                        Gamepad1.getLeftX(),
                        Gamepad1.getLeftY(),
                        Gamepad1.getRightX(),
                        false
                );
            } else {
                BarkMecanumDrive BarkDrive = new BarkMecanumDrive(hardwareMap);
                drive.driveFieldCentric(
                        Gamepad1.getLeftX(),
                        Gamepad1.getLeftY(),
                        Gamepad1.getRightX(),
                        Math.toDegrees(BarkDrive.getPoseEstimate().getHeading()),
                        false
                );
            }

        }
    }

}