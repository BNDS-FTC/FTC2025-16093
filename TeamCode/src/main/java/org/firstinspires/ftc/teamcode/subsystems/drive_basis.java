package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drive_basis extends SubsystemBase {

    private final MecanumDrive drive;
    private boolean FIELD_CENTRIC = false; // Toggle for field-centric control

    /**
     * Creates a new MecanumDrive subsystem.
     * @param fieldCentric Control mode
     */
    public drive_basis(boolean fieldCentric) {
        // Initialize the MecanumDrive using FTCLib's Motor class
        drive = new MecanumDrive(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftBack"),
                new Motor(hardwareMap, "rightBack")
        );
        this.FIELD_CENTRIC=fieldCentric;
    }

    @Override
    public void periodic(){
        /*
        * Todo:
        *  1.read values from hardware
        *  2.apply PID
        *  3.call drive() with parameters
        * */

    }
    /**
     * Drives the robot using either robot-centric or field-centric control.
     *
     * @param leftX   Strafe input.
     * @param leftY   Forward/backward input.
     * @param rightX  Rotation input.     
     * @param heading The current heading of the robot (used for field-centric).
     */
    public void drive(double leftX, double leftY, double rightX, double heading) {
        if (!FIELD_CENTRIC) {
            // Robot-centric control
            drive.driveRobotCentric(leftX,leftY,rightX,false);
        } else {
            drive.driveFieldCentric(leftX,leftY,rightX,heading,false);
        }
    }
}