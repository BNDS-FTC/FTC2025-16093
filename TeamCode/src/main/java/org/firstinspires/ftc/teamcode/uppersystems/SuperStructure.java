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

import org.firstinspires.ftc.teamcode.references.SSValues;


@Config
public class SuperStructure {
    private DcMotorEx mArm = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    private Servo mIntakeLeft; // continuous
    private Servo mIntakeRight;// continuous
    private Servo mWrist;
    private Servo mGrab;

    private TouchSensor mTouchSensor;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.005, 0.0003, 0.0003);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients lSlidePidConf = new PIDCoefficients(0.05, 0, 0);
    private final PIDFController lSlidePidCtrl;
    public static PIDCoefficients rSlidePidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController rSlidePidCtrl;
//    private Servo mClawLeft = null;
//    private Servo mClawRight = null;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure(LinearOpMode opMode){
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);
        lSlidePidCtrl = new PIDFController(lSlidePidConf);
        rSlidePidCtrl = new PIDFController(rSlidePidConf);

        mArm = hardwareMap.get(DcMotorEx.class,"arm");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
//        mClawLeft = hardwareMap.get(Servo.class,"clawLeft");
//        mClawRight = hardwareMap.get(Servo.class,"clawRight");

        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mArm.setDirection(DcMotorSimple.Direction.REVERSE);

        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");
        mWrist = hardwareMap.get(Servo.class,"wrist");
        mGrab = hardwareMap.get(Servo.class,"grab");

        mTouchSensor = hardwareMap.get(TouchSensor.class,"touch");

//        mIntakeRight.setDirection(Servo.Direction.REVERSE);

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Arm
    private int armTargetPosition;
    private int armError;
    public void setArmPosition(int pos){
        armTargetPosition = pos;
        armError = armTargetPosition - mArm.getCurrentPosition();
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(Math.abs(armError) <= 50) {
            armPidCtrl.setOutputBounds(-0.2, 0.2);
        }else if(mArm.getCurrentPosition() <= 500 && pos <= mArm.getCurrentPosition()) {
            armPidCtrl.setOutputBounds(-0.1, 0.1);
        }else if(mArm.getCurrentPosition() <= 1200 && pos <= mArm.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.2,0.2);
        }else if(mArm.getCurrentPosition() < 1400 && pos <= mArm.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.5,0.5);
        }else if(pos <= mArm.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.5,0.5);
        }else{
            armPidCtrl.setOutputBounds(-0.8,0.8);
        }

    }

//    public void setArmByP(int pos, double power){
//        armTargetPosition = pos;
//        mArm.setTargetPosition(pos);
//        mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        mArm.setPower(power);
//    }

    public void update() {
        mSlideRight.setPower(rSlidePidCtrl.update(mSlideRight.getCurrentPosition()-slideTargetPosition));
        mSlideLeft.setPower(lSlidePidCtrl.update(mSlideLeft.getCurrentPosition()-slideTargetPosition));
        mArm.setPower(armPidCtrl.update(mArm.getCurrentPosition() - armTargetPosition));
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Slide
    public int slideTargetPosition;
    public void setSlidePosition(int pos){
        slideTargetPosition = pos;
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(mTouchSensor.isPressed()){
            lSlidePidCtrl.setOutputBounds(0,0);
            rSlidePidCtrl.setOutputBounds(0,0);
        }//This is very fishy code. Needs to be checked.

        if(getSlidePosition() <= 800 && pos <= getSlidePosition()){
            lSlidePidCtrl.setOutputBounds(-0.2,0.2);
            rSlidePidCtrl.setOutputBounds(-0.2,0.2);
        }else if(getSlidePosition() < 1400 && pos >= getSlidePosition()){
            lSlidePidCtrl.setOutputBounds(-0.9,0.9);
            rSlidePidCtrl.setOutputBounds(-0.9,0.9);
        }else{
            lSlidePidCtrl.setOutputBounds(-0.8,0.8);
            rSlidePidCtrl.setOutputBounds(-0.8,0.8);
        }
    }

    public void resetSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setPower(-0.3);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setPower(-0.3);

        opMode.sleep(300);

        mSlideRight.setPower(0);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideLeft.setPower(0);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //Intake Action
    public void intakeFar(){
        setArmPosition(SSValues.ARM_INTAKE_FAR);
        sleep(500);
        //setArmByPower(SSValues.ARM_INTAKE_FAR,1);
        mWrist.setPosition(SSValues.WRIST_DROP);
        setSlidePosition(SSValues.SLIDE_MAX);
    }
    public void intakeNear(){
        setArmPosition(SSValues.ARM_INTAKE_NEAR);
        mWrist.setPosition(SSValues.WRIST_DROP);
        setSlidePosition(SSValues.SLIDE_MIN);
    }

    // Release Action
    public void releaseHigh(){
        setArmPosition(SSValues.ARM_UP);
        mWrist.setPosition(SSValues.WRIST_INTAKE);
        setSlidePosition(SSValues.SLIDE_MAX);
    }

    //Default pose
    public void resetPos(){
        setArmPosition(SSValues.ARM_DEFAULT);
        mWrist.setPosition(SSValues.WRIST_DEFAULT);
        setSlidePosition(SSValues.SLIDE_MIN);
    }

    //Intake Sequences
    public void rollIn(){
        mIntakeLeft.setPosition(SSValues.CONTINUOUS_SPIN);
        mIntakeRight.setPosition(SSValues.CONTINUOUS_SPIN);
    }
    public void rollOut(){
        mIntakeLeft.setPosition(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        mIntakeRight.setPosition(SSValues.CONTINUOUS_SPIN_OPPOSITE);
    }
    public void rollStop(){
        mIntakeLeft.setPosition(SSValues.CONTINUOUS_STOP);
        mIntakeRight.setPosition(SSValues.CONTINUOUS_STOP);
    }

    //Wrist Sequences
    public void wristIntake(){
        mWrist.setPosition(SSValues.WRIST_INTAKE);
    }
    public void wristDrop(){
        mWrist.setPosition(SSValues.WRIST_DROP);
    }

    public void grabOpen(){
        mGrab.setPosition(SSValues.GRAB_OPEN);
    }
    public void grabClose(){
        mGrab.setPosition(SSValues.GRAB_CLOSED);
    }

    //Getters and Setters
    public int getArmPosition(){
        return mArm.getCurrentPosition();
    }
    public int getSlideLeftPosition(){
        return mSlideLeft.getCurrentPosition();
    }
    public int getSlideRightPosition(){
        return mSlideRight.getCurrentPosition();
    }
    public int getSlidePosition(){
        return getSlideLeftPosition()+getSlideRightPosition()/2;
    }
    public double getArmPower(){
        return mArm.getPower();
    }
    public int getArmTargetPosition(){
        return armTargetPosition;
    }
    public int getSlideTargetPosition(){
        return getSlideTargetPosition();
    }
    public boolean getTouchSensorPressed(){
        return mTouchSensor.isPressed();
    }

    public void sleep(int sleepTime) {
        long end = System.currentTimeMillis() + sleepTime;
        while (opMode.opModeIsActive() && end > System.currentTimeMillis() && updateRunnable != null) {
            updateRunnable.run();
        }
    }
}
