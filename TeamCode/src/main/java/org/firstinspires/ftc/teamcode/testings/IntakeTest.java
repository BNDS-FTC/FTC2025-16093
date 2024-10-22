package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Intake Test")
@Config
public class IntakeTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static boolean read_only = false;
    public static double pos = 0.5;

    private Servo mIntakeLeft = null; // continuous
    private Servo mIntakeRight = null;// continuous

    @Override
    public void runOpMode() {

        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");

        mIntakeRight.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if (!read_only) {
                setIntakeSpin(pos);
                telemetry_M.addData("Current State", "Intake");
//                telemetry_M.addData("rightfront", servo1.getPosition());
                telemetry_M.update();
            }
        }
    }

    public void setIntakeSpin(double value){
        mIntakeLeft.setPosition(value);
        mIntakeRight.setPosition(value);
    }
}
