package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Intake Test",group = "Testing")
@Config
public class IntakeTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static boolean read_only = false;
    public static State state;
    public static State previousState;
    public static double pos = 0.5;
    public enum State{
        SPIN,
        STOP,
        OPPOSITE
    }

    private Servo mIntakeLeft = null; // continuous
    private Servo mIntakeRight = null;// continuous
    private Servo mGrab = null;

    @Override
    public void runOpMode() {

        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");
        mGrab = hardwareMap.get(Servo.class,"grab");

//        mIntakeRight.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper){ //Spin in
                switchState(State.OPPOSITE);
                pos = 0.8;
            }else if(gamepad1.left_bumper){ //Spin out
                switchState(State.SPIN);
                pos = 0;
            }else{
                switchState(State.STOP);
                if(previousState == State.OPPOSITE){
                    pos = 0.76;
                }else if(previousState == State.SPIN){
                    pos = 0.3;
                }else{
                    while(pos != 0.5){
                        if(pos > 0.5){
                            pos -= 0.1;
                        }else{
                            pos += 0.1;
                        }
                    }
                }
            }
            setIntakeSpin(pos);
            mGrab.setPosition(0.32);

//            if (!read_only) {
//                setIntakeSpin(pos);
//                telemetry_M.addData("Current State", "Intake");
////                telemetry_M.addData("rightfront", servo1.getPosition());
//                telemetry_M.update();
//            }

            telemetry.addData("State: ", state);
            telemetry.addData("Pos: ", pos);
            telemetry.update();
        }
    }

    public void switchState(State s){
        previousState = state;
        state = s;
    }

    public void setIntakeSpin(double value){
        mIntakeLeft.setPosition(value);
        mIntakeRight.setPosition(value);
    }

}
