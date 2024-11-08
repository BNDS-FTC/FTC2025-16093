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
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    private Servo mIntakeLeft; // continuous
    private Servo mIntakeRight;// continuous
    private Servo mWrist;
    private Servo mGrab;

    private TouchSensor mTouchSensor;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.09, 0, 0);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients lSlidePidConf = new PIDCoefficients(0.0025, 0.0004, 0.00013);
    private final PIDFController lSlidePidCtrl;
    public static PIDCoefficients rSlidePidConf = new PIDCoefficients(0.0025, 0.0004, 0.00013);
    private final PIDFController rSlidePidCtrl;
    private Servo mGrabLeft = null;
    private Servo mGrabRight = null;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure(LinearOpMode opMode, Runnable updateRunnable){
        this.opMode = opMode;
        this.updateRunnable = updateRunnable;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);
        lSlidePidCtrl = new PIDFController(lSlidePidConf,0,0,0);
        rSlidePidCtrl = new PIDFController(rSlidePidConf);

        mArm = hardwareMap.get(DcMotorEx.class,"arm");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mGrabLeft = hardwareMap.get(Servo.class,"grabLeft");
        mGrabRight = hardwareMap.get(Servo.class,"grabRight");

        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mArm.setDirection(DcMotorSimple.Direction.REVERSE);

        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");
        mWrist = hardwareMap.get(Servo.class,"wrist");
        mGrab = hardwareMap.get(Servo.class,"grab");

        mTouchSensor = hardwareMap.get(TouchSensor.class,"touch");

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mGrab.setDirection(Servo.Direction.REVERSE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lSlidePidCtrl.setOutputBounds(-0.8, 0.8);
        rSlidePidCtrl.setOutputBounds(-0.8, 0.8);
    }

    public void setSlidesByP(int pos, double power){
        mSlideLeft.setTargetPosition(pos);
        mSlideRight.setTargetPosition(pos);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mSlideLeft.setPower(power);
        mSlideRight.setPower(power);
    }

    public void resetSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setPower(-0.3);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setPower(-0.3);

        opMode.sleep(50);

        mSlideRight.setPower(0);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideLeft.setPower(0);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlidesToRunByPower(){
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setSlidesByPower(double power){
        mSlideLeft.setPower(power);
        mSlideRight.setPower(power);
    }

    public void setIntake(double val){
        mIntakeLeft.setPosition(val);
        mIntakeRight.setPosition(val);
    }
    public void setWristPos(double pos){
        mWrist.setPosition(pos);
    }
    public void setGrabPos(double pos){
        mGrab.setPosition(pos);
    }



    ///////////////////////////////////GETTERS AND SETTERS//////////////////////////////////////////
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
        return (getSlideLeftPosition()+getSlideRightPosition())/2;
    }
    public double getWristPosition(){
        return mWrist.getPosition();
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

    //This is not being used because it's not very good? As in, it doesn't work the way you think it would.
    public void sleep(int sleepTime) {

        long end = System.currentTimeMillis() + sleepTime;
        while (opMode.opModeIsActive() && end > System.currentTimeMillis() && updateRunnable != null) {
            updateRunnable.run();
        }
    }
}
