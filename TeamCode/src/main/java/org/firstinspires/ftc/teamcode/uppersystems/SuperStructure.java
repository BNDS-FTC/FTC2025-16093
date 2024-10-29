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

import org.firstinspires.ftc.teamcode.references.SSValues;


@Config
public class SuperStructure {
    private DcMotorEx mArm = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients slidePidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController slidePidCtrl;
//    private Servo mClawLeft = null;
//    private Servo mClawRight = null;

    private Servo mIntakeLeft; // continuous
    private Servo mIntakeRight;// continuous
    private Servo mWrist;
    private Servo mGrab;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure(LinearOpMode opMode){
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);
        slidePidCtrl = new PIDFController(slidePidConf);

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

        mIntakeRight.setDirection(Servo.Direction.REVERSE);

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
    public void setArmPosition(int pos){
        armTargetPosition = pos;
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(mArm.getCurrentPosition() <= 800 && pos <= mArm.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.3,0.3);
        }else if(mArm.getCurrentPosition() < 1400 && pos >= mArm.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.9,0.9);
        }else if(pos >= mArm.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.5,0.5);
        }else{
            armPidCtrl.setOutputBounds(-0.8,0.8);
        }
    }
    public void update() {
        mArm.setPower(armPidCtrl.update(mArm.getCurrentPosition() - armTargetPosition));
        mSlideRight.setPower(slidePidCtrl.update(mSlideRight.getCurrentPosition()-slideTargetPosition));
        mSlideLeft.setPower(slidePidCtrl.update(mSlideLeft.getCurrentPosition()-slideTargetPosition));
    }

    //Slide
    public int slideTargetPosition;
    public void setSlidePosition(int pos){
        slideTargetPosition = pos;
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(getSlidePosition() <= 800 && pos <= getSlidePosition()){
            armPidCtrl.setOutputBounds(-0.3,0.3);
        }else if(mArm.getCurrentPosition() < 1400 && pos >= mArm.getCurrentPosition()){
            armPidCtrl.setOutputBounds(-0.9,0.9);
        }else{
            slidePidCtrl.setOutputBounds(-0.8,0.8);
        }
    }
    public void resetSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setPower(-0.3);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setPower(-0.3);

        long end = System.currentTimeMillis() + 300;
        while (end > System.currentTimeMillis()) {
            opMode.idle();
        }
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
        mWrist.setPosition(SSValues.WRIST_INTAKE);
        setSlidePosition(SSValues.SLIDE_MAX);
    }
    public void intakeNear(){
        setArmPosition(SSValues.ARM_INTAKE_NEAR);
        mWrist.setPosition(SSValues.WRIST_INTAKE);
        setSlidePosition(SSValues.SLIDE_MIN);
    }

    // Release Action
    public void releaseHigh(){
        setArmPosition(SSValues.ARM_UP);
        mWrist.setPosition(SSValues.WRIST_DROP);
        setSlidePosition(SSValues.SLIDE_MAX);
    }

    //Default pose
    public void resetPos(){
        setArmPosition(SSValues.ARM_DEFAULT);
        mWrist.setPosition(SSValues.WRIST_DEFAULT);
        setSlidePosition(SSValues.SLIDE_MIN);
    }

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

}
