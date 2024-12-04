package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.references.ServoPWMControl;
import org.firstinspires.ftc.teamcode.testings.ArmAdjustment;

public class IntakeSubsystem extends SubsystemBase {
    private final Servo mIntakeLeft; // continuous
    private final Servo mIntakeRight;// continuous
    public ServoPWMControl controlLeft;
    public ServoPWMControl controlRight;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");
        controlLeft = new ServoPWMControl(mIntakeLeft);
        controlRight = new ServoPWMControl(mIntakeRight);
    }

    public void stopIntake(){
        controlRight.setStatus(false);
        controlLeft.setStatus(false);
    }
    public void startIntake(){
        controlRight.setStatus(true);
        controlLeft.setStatus(true);
    }
    public void setIntake(double val){
        mIntakeLeft.setPosition(val);
        mIntakeRight.setPosition(val);
    }
}