package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class drive_basis {
    public static final GamepadEx Gamepad1 = new GamepadEx(gamepad1);
    public static GamepadEx Gamepad2 = new GamepadEx(gamepad2);
    public static ServoEx mIntakeLeft = new SimpleServo(hardwareMap, "intakeLeft",0,255);
    public static ServoEx mIntakeRight = new SimpleServo(hardwareMap, "intakeRight",0,255);
    public static ServoEx Wrist = new SimpleServo(hardwareMap, "wrist",0,255);
    public static ServoEx Grab = new SimpleServo(hardwareMap, "grab",0,255);
    public static ServoEx clawLeft = new SimpleServo(hardwareMap, "clawLeft",0,255);
    public static ServoEx clawRight = new SimpleServo(hardwareMap, "clawRight",0,255);
    public static Motor mleftFront = new Motor(hardwareMap, "leftFront");
}
