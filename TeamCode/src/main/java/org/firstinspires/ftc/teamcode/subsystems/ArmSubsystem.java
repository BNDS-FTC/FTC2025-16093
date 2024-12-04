package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.testings.ArmAdjustment;

public class ArmSubsystem extends SubsystemBase {

    private final DcMotorEx mArm;
    private final TouchSensor mTouchSensor;

    int armTargetPosition;
    public int armOffset;

    public ArmSubsystem(HardwareMap hardwareMap, int armOffset) {
        mArm = hardwareMap.get(DcMotorEx.class,"arm");
        mArm.setDirection(DcMotorSimple.Direction.REVERSE);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mTouchSensor = hardwareMap.get(TouchSensor.class, "touch");


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(getArmTargetPosition() < getArmPosition()){
            mArm.setPower(Math.max(ArmAdjustment.armMinPower, Math.min(ArmAdjustment.coefficient*Math.cos(getArmPosition()*Math.PI/2000),1)));
        }

        if((Math.abs(getArmError()) < 30)){
            mArm.setPower(0);
        }

        if(mTouchSensor.isPressed()){
            resetArmEncoder();
        }
    }

    public void armStop(){
        setArmByP(getArmPosition(),0);
    }

    public void setArmByP(int pos, double power){
        armTargetPosition = pos;
        mArm.setTargetPosition(pos);
        mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mArm.setPower(power);
    }

    public void resetArmEncoder(){
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArmByPower(double power){
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArm.setPower(power);
    }

    public void setArmPower(double power){
        mArm.setPower(power);
    }

    public int getArmPosition(){
        return mArm.getCurrentPosition();
    }
    public double getArmPower(){
        return mArm.getPower();
    }
    public int getArmTargetPosition(){
        return armTargetPosition;
    }
    public int getArmError(){return armTargetPosition-mArm.getCurrentPosition();}
}