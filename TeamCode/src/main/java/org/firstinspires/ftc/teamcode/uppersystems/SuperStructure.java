package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Config
public class SuperStructure {
    private DcMotorEx mArm = null;
    private DcMotorEx mSlideRight = null;

    private Servo mIntakeLeft; // continuous
    private Servo mIntakeRight;// continuous
    private Servo Wrist;
    private Servo Grab;
    private Servo clawLeft;
    private Servo clawRight;

    private TouchSensor mTouchSensor;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.09, 0, 0);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients rSlidePidConf = new PIDCoefficients(0.0025, 0.0004, 0.00013);
    private final PIDFController rSlidePidCtrl;
    private final LinearOpMode opMode;
    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure(LinearOpMode opMode, Runnable updateRunnable){

        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);
        rSlidePidCtrl = new PIDFController(rSlidePidConf);

        mArm = hardwareMap.get(DcMotorEx.class,"arm");

        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mArm.setDirection(DcMotorSimple.Direction.REVERSE);

        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");
        Wrist = hardwareMap.get(Servo.class,"wrist");
        Grab = hardwareMap.get(Servo.class,"grab");

        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRight = hardwareMap.get(Servo.class,"clawRight");

        mTouchSensor = hardwareMap.get(TouchSensor.class,"touch");
//
        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Grab.setDirection(Servo.Direction.REVERSE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
//        mSlideRight.setPower(rSlidePidCtrl.update(mSlideRight.getCurrentPosition()-slideTargetPosition));
//        mSlideLeft.setPower(lSlidePidCtrl.update(mSlideLeft.getCurrentPosition()-slideTargetPosition));
//        if(Math.abs(mArm.getCurrentPosition() - armTargetPosition) < 30){
//            mArm.setPower(0);
//        }else{
//            mArm.setPower(armPidCtrl.update(mArm.getCurrentPosition() - armTargetPosition));
//        }
    }

    ///////////////////////////////////////ARM//////////////////////////////////////////////////////
    private int armTargetPosition;
    private int armError;
    public void setArmPosition(int pos, double power){
        mArm.setPower(power);
        armTargetPosition = pos;
        armError = mArm.getCurrentPosition() - armTargetPosition;
        if(armError>0){
            armPidCtrl.setOutputBounds(-0.7,0.7);
        }else{
            armPidCtrl.setOutputBounds(-0.9,0.9);
        }
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void setArmToRunByPower(){
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setArmByPower(double power){
        mArm.setPower(power);
    }

    ///////////////////////////////////////SLIDES//////////////////////////////////////////////////
    public int slideTargetPosition;
    public void setSlidePosition(int pos) {
        slideTargetPosition = pos;
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSlidePidCtrl.setOutputBounds(-0.8, 0.8);
    }

    public void setSlidesByP(int pos, double power){
        mSlideRight.setTargetPosition(pos);
        if(mSlideRight.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        mSlideRight.setPower(power);
    }

    public void resetSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setPower(-0.3);

        opMode.sleep(50);

        mSlideRight.setPower(0);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlidesByPower(double power){
        if(mSlideRight.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        mSlideRight.setPower(power);
    }

    public void setIntake(double val){
        mIntakeLeft.setPosition(val);
        mIntakeRight.setPosition(val);
    }
    public void setWristPos(double pos){
        Wrist.setPosition(pos);
    }
    public void setGrabPos(double pos){
        Grab.setPosition(pos);
    }
    public void setClawLeftPos(double pos){clawLeft.setPosition(pos);}
    public void setClawRightPos(double pos){clawRight.setPosition(pos);}



    ///////////////////////////////////GETTERS AND SETTERS//////////////////////////////////////////
    public int getArmPosition(){
        return mArm.getCurrentPosition();
    }

    public int getSlideRightPosition(){
        return mSlideRight.getCurrentPosition();
    }
    public int getSlidePosition(){
        return mSlideRight.getCurrentPosition();
    }
    public double getWristPosition(){
        return Wrist.getPosition();
    }
    public double getArmPower(){
        return mArm.getPower();
    }
    public int getArmTargetPosition(){
        return armTargetPosition;
    }
    public int getSlideTargetPosition(){
        return slideTargetPosition;
    }
    public boolean getTouchSensorPressed(){
        return mTouchSensor.isPressed();
    }
    public double getClawLeft(){return clawLeft.getPosition();}
    public double getClawRight(){return clawRight.getPosition();}
}
