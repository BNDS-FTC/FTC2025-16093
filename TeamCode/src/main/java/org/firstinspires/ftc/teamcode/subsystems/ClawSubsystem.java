package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.testings.ArmAdjustment;

public class ClawSubsystem extends SubsystemBase {

    private final Servo mClawLeft;
    private final Servo mClawRight;

    private clawState state;


    public ClawSubsystem(HardwareMap hardwareMap) {
        mClawLeft = hardwareMap.get(Servo.class,"clawLeft");
        mClawRight = hardwareMap.get(Servo.class,"clawRight");

    }

    public void setClawPos(clawState s){
        state = s;
        if(state == clawState.OPEN){
            mClawLeft.setPosition(SSValues.CLAW_LEFT_OPEN);
            mClawRight.setPosition(SSValues.CLAW_RIGHT_OPEN);
        }else if(state == clawState.CLOSED){
            mClawLeft.setPosition(SSValues.CLAW_LEFT_CLOSE);
            mClawRight.setPosition(SSValues.CLAW_RIGHT_CLOSE);
        }
    }

    public void setClawLeftPos(double pos){mClawLeft.setPosition(pos);}
    public void setClawRightPos(double pos){mClawRight.setPosition(pos);}

    public boolean isClawOpen(){
        if(state == clawState.OPEN){
            return true;
        }else{
            return false;
        }
    }

    public enum clawState{
        OPEN,
        CLOSED
    }
}