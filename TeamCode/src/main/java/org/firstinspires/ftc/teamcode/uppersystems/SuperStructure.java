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

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.0002, 0.00013);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients lSlidePidConf = new PIDCoefficients(0.0025, 0.0002, 0.00013);
    private final PIDFController lSlidePidCtrl;
    public static PIDCoefficients rSlidePidConf = new PIDCoefficients(0.0025, 0.0002, 0.00013);
    private final PIDFController rSlidePidCtrl;
//    private Servo mClawLeft = null;
//    private Servo mClawRight = null;

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
        //mArm.setPower(armPidCtrl.update(mArm.getCurrentPosition() - armTargetPosition));
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Arm
    private int armTargetPosition;
    private int armError;
    public void setArmPosition(int pos){
        armTargetPosition = pos;
        armError = mArm.getCurrentPosition() - armTargetPosition;
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(armError > 0){
            armPidCtrl.setOutputBounds(-0.2, 0.2);
        }else{
            armPidCtrl.setOutputBounds(-0.8,0.8);
        }

//        if(Math.abs(armError) <= 50) {
//            armPidCtrl.setOutputBounds(-0.2, 0.2);
//        }else if(mArm.getCurrentPosition() <= 500 && pos <= mArm.getCurrentPosition()) {
//            armPidCtrl.setOutputBounds(-0.1, 0.1);
//        }else if(mArm.getCurrentPosition() <= 1200 && pos <= mArm.getCurrentPosition()){
//            armPidCtrl.setOutputBounds(-0.2,0.2);
//        }else if(mArm.getCurrentPosition() < 1400 && pos <= mArm.getCurrentPosition()){
//            armPidCtrl.setOutputBounds(-0.5,0.5);
//        }else if(pos <= mArm.getCurrentPosition()){
//            armPidCtrl.setOutputBounds(-0.5,0.5);
//        }else{
//            armPidCtrl.setOutputBounds(-0.8,0.8);
//        }

    }

    boolean armZeroPower = false;
    public void setArmZeroPower(){
        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armZeroPower = true;
    }
    public void giveArmPowerAgain(){
        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armZeroPower = false;
    }

    public void setSlidesByP(int pos, double power){
        mSlideLeft.setTargetPosition(pos);
        mSlideRight.setTargetPosition(pos);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mSlideLeft.setPower(power);
        mSlideRight.setPower(power);
    }

    public void setArmByP(int pos, double power){
        armTargetPosition = pos;
        mArm.setTargetPosition(pos);
        mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mArm.setPower(power);
    }

    //Slide
    public int slideTargetPosition;
    public void setSlidePosition(int pos){
        //if(Math.abs(armTargetPosition-mArm.getCurrentPosition()) < 300){
            slideTargetPosition = pos;
            mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(mTouchSensor.isPressed()){
                lSlidePidCtrl.setOutputBounds(0,0);
                rSlidePidCtrl.setOutputBounds(0,0);
            }//This doesn't really do anything as of now because a. the arm doesn't contact the sensor correctly
            //b. by the time it contacts the sensor it's not really useful anymore?

            if(getSlidePosition() <= 200 && pos <= getSlidePosition()){
                lSlidePidCtrl.setOutputBounds(-0.4,0.4);
                rSlidePidCtrl.setOutputBounds(-0.4,0.4);
            }else if(getSlidePosition() < 1400 && pos >= getSlidePosition()){
                lSlidePidCtrl.setOutputBounds(-0.9,0.9);
                rSlidePidCtrl.setOutputBounds(-0.9,0.9);
            }else{
                lSlidePidCtrl.setOutputBounds(-0.9,0.9);
                rSlidePidCtrl.setOutputBounds(-0.9,0.9);
            }
        //}
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

    //Intake Servos
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

    public void resetArmEncoder(){
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void sleep(int sleepTime) {

        long end = System.currentTimeMillis() + sleepTime;
        while (opMode.opModeIsActive() && end > System.currentTimeMillis() && updateRunnable != null) {
            updateRunnable.run();
        }
    }
}
