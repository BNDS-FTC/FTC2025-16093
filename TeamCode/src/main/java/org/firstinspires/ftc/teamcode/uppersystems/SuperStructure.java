package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.ServoPWMControl;
import org.firstinspires.ftc.teamcode.testings.ArmAdjustment;

import java.util.List;

/**
 * 希望它不要爆掉...如果爆掉了就重启吧!
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                   O\  =  /O
 *                ____/`---'\____
 *              .'  \\|     |//  `.
 *             /  \\|||  :  |||//  \
 *            /  _||||| -:- |||||-  \
 *            |   | \\\  -  /// |   |
 *            | \_|  ''\---/''  |   |
 *            \  .-\__  `-`  ___/-. /
 *          ___`. .'  /--.--\  `. . __
 *       ."" '<  `.___\_<|>_/___.'  >'"".
 *      | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *      \  \ `-.   \_ __\ /__ _/   .-` /  /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *             佛祖保佑       永无BUG
 **/

@Config
public class SuperStructure {
    private final DcMotorEx mArm;
    private final DcMotorEx mSlideRight;
    private final DcMotorEx mSlideLeft;

    private final Servo mIntakeLeft; // continuous
    private final Servo mIntakeRight;// continuous
    private final Servo Wrist;
    private final Servo Grab;
    private final Servo clawLeft;
    private final Servo clawRight;

    Sequences sequence;
    Sequences previousSequence;

    private final TouchSensor mTouchSensor;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.09, 0, 0);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients rSlidePidConf = new PIDCoefficients(0.025, 0, 0);
    private final PIDFController rSlidePidCtrl;
    public static PIDCoefficients lSlidePidConf = new PIDCoefficients(0.025, 0, 0);
    private final PIDFController lSlidePidCtrl;
    public static PIDCoefficients rSlidePidConfVertical = new PIDCoefficients(0.008, 0, 0);
    private final PIDFController rSlidePidCtrlVertical;
    public static PIDCoefficients lSlidePidConfVertical = new PIDCoefficients(0.008, 0, 0);
    private final PIDFController lSlidePidCtrlVertical;
    public ServoPWMControl controlLeft = null;
    public ServoPWMControl controlRight = null;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;
//    private XCYBoolean slideZeroVelocity;

    public int armOffset;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure(LinearOpMode opMode, Runnable updateRunnable, int armOffset){
        Action.clearActions();

        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.updateRunnable = updateRunnable;
        armPidCtrl = new PIDFController(armPidConf);
        rSlidePidCtrl = new PIDFController(rSlidePidConf);
        lSlidePidCtrl = new PIDFController(lSlidePidConf);
        rSlidePidCtrlVertical = new PIDFController(rSlidePidConfVertical);
        lSlidePidCtrlVertical = new PIDFController(lSlidePidConfVertical);

        mArm = hardwareMap.get(DcMotorEx.class,"arm");

        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        mSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Grab.setDirection(Servo.Direction.REVERSE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controlLeft = new ServoPWMControl(mIntakeLeft);
        controlRight = new ServoPWMControl(mIntakeRight);


//        slideZeroVelocity = new XCYBoolean(()->mSlideLeft.getVelocity() == 0);

        this.sequence = Sequences.RUN;
        this.previousSequence = Sequences.RUN;
        this.armOffset = armOffset;
    }


    public void update() {
//        if(TeleOp16093.slideMode == 0){
//        if ((Math.abs(mArm.getCurrentPosition() - SSValues.ARM_UP) < 30) && slideTargetPosition < mSlideLeft.getCurrentPosition()) {
//            mSlideRight.setPower(rSlidePidCtrlVertical.update(mSlideLeft.getCurrentPosition() - slideTargetPosition));
//            mSlideLeft.setPower(lSlidePidCtrlVertical.update(mSlideLeft.getCurrentPosition() - slideTargetPosition));
//        } else {
//            mSlideRight.setPower(rSlidePidCtrl.update(mSlideLeft.getCurrentPosition() - slideTargetPosition));
//            mSlideLeft.setPower(lSlidePidCtrl.update(mSlideLeft.getCurrentPosition() - slideTargetPosition));
//        }
        //            mArm.setPower(armPidCtrl.update(mArm.getCurrentPosition() - armTargetPosition));
        if(Math.abs(getSlideError())<30){
            if((mArm.getCurrentPosition() >= (SSValues.ARM_UP - 100)) && slideTargetPosition == SSValues.SLIDE_MAX){
                mSlideLeft.setPower(0.3);
                mSlideRight.setPower(0.3);
            }
            else{
                mSlideLeft.setPower(0.1);
                mSlideRight.setPower(0.1);
            }
        }

        if(getArmTargetPosition() < getArmPosition()){
            mArm.setPower(Math.max(ArmAdjustment.armMinPower, Math.min(ArmAdjustment.coefficient*Math.cos(getArmPosition()*Math.PI/2000),1)));
        }
    }


    // Switches the sequence to a new state and stores the previous one
    public void switchSequence(Sequences s) {
        previousSequence = sequence;
        sequence = s;
    }

    // Enum for sequence states
    public enum Sequences {
        RUN,
        INTAKE_FAR,
        INTAKE_NEAR,
        HIGH_BASKET,
        HANG,
        CUSTOM_INTAKE,
        LOW_BASKET,
        HIGH_CHAMBER,
        ASCENT
        //Etc.
    }

    ///////////////////////////////////////ARM//////////////////////////////////////////////////////
    private int armTargetPosition;
    private int armError;
    public void setArmPosition(int pos, double power){
        mArm.setPower(power);
        armTargetPosition = pos;
        armError = getArmPosition() - armTargetPosition;
        if(armError>0){
            armPidCtrl.setOutputBounds(-1,1);
        }else{
            armPidCtrl.setOutputBounds(-1,1);
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

    public void setArmByPower(double power){
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArm.setPower(power);
    }

    ////////////////////////////////////////SLIDES//////////////////////////////////////////////////
    public int slideTargetPosition;
    public void setSlidePosition(int pos, double power) {
        slideTargetPosition = pos;
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(slideTargetPosition < mSlideLeft.getCurrentPosition() && Math.abs(getArmPosition() - SSValues.ARM_UP) < 20){
            rSlidePidCtrl.setOutputBounds(-0.3, 0.3);
            lSlidePidCtrl.setOutputBounds(-0.3, 0.3);
        }else if(power == 0){
            rSlidePidCtrl.setOutputBounds(0,0);
            lSlidePidCtrl.setOutputBounds(0,0);
        } else{
            rSlidePidCtrl.setOutputBounds(-1, 1);
            lSlidePidCtrl.setOutputBounds(-1, 1);
        }

    }

    public void setSlidesByP(int pos, double power){
        mSlideRight.setTargetPosition(pos);
        mSlideLeft.setTargetPosition(pos);
        if(mSlideRight.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        mSlideRight.setPower(power);
        mSlideLeft.setPower(power);
    }

    public void resetSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setPower(-0.3);
        mSlideLeft.setPower(-0.3);

        opMode.sleep(50);

        mSlideRight.setPower(0);
        mSlideLeft.setPower(0);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetSlideEncoder(){
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSlidesByPower(double power){
        if(mSlideRight.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        rSlidePidCtrl.setOutputBounds(-0, 0);
        lSlidePidCtrl.setOutputBounds(-0, 0);
        mSlideRight.setPower(power);
        mSlideLeft.setPower(power);
    }

    public void resetSlideDuringTeleOp(){
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void stopIntake(){
        controlRight.setStatus(false);
        controlLeft.setStatus(false);
    }
    public void startIntake(){
        controlRight.setStatus(true);
        controlLeft.setStatus(true);
    }

    public void runFor(int ms){
        long startTime = System.currentTimeMillis();
        boolean stopped = false;
        controlRight.setStatus(true);
        controlLeft.setStatus(true);
        if(startTime+ms < System.currentTimeMillis() && !stopped){
            controlRight.setStatus(false);
            controlLeft.setStatus(false);
            stopped = true;
        }
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
    public int getSlideLeftPosition(){
        return mSlideLeft.getCurrentPosition();
    }
    public int getSlidesPosition(){
        return (mSlideLeft.getCurrentPosition()+mSlideRight.getCurrentPosition())/2;
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
    public double getSlidePower(){
        return mSlideLeft.getPower();
    }
    public double getSlideVelocity(){return mSlideLeft.getVelocity();}
    public double getSlideError(){
        return (double) (mSlideLeft.getTargetPosition() - mSlideLeft.getCurrentPosition() + mSlideRight.getTargetPosition() - mSlideRight.getCurrentPosition())/2;
    }
    public boolean getTouchSensorPressed(){
        return mTouchSensor.isPressed();
    }
    public double getClawLeft(){return clawLeft.getPosition();}
    public double getClawRight(){return clawRight.getPosition();}
    public Sequences getSequence(){return sequence;}
    public Sequences getPreviousSequence(){return previousSequence;}
//    public boolean getSlideVelocityToZero(){
//        return slideZeroVelocity.toTrue();
//    }

}