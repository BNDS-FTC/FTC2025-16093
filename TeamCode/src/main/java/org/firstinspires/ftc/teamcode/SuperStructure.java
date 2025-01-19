package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.actions.Action;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
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
    private final Servo mWrist;
    private final Servo mGrab;
    private final Servo clawLeft;
    private final Servo clawRight;
    private final Servo slideLock;

    Sequences sequence;
    Sequences previousSequence;

    public final TouchSensor mTouchSensor;

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
//    public ServoPWMControl controlLeft = null;
//    public ServoPWMControl controlRight = null;

//    private final ColorSensor color;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;
//    private XCYBoolean slideZeroVelocity;

    public int armOffset;
    double currentArmPower, currentSlideLeftPower, currentSlideRightPower;
    double currentWristPos, currentGrabPos;
    int currentArmPos, currentSlideLeftPos, currentSlideRightPos;
    boolean currentTouchSensorState = true;
    DcMotor.RunMode currentArmMode, currentSlideMode;

    private final List<Integer> cachedColor = new ArrayList<>(Arrays.asList(0,0,0,-1));
    private long lastRead =0;
    private boolean cachedRes = false;

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
        mSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mArm.setDirection(DcMotorSimple.Direction.REVERSE);


        mIntakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        mIntakeRight = hardwareMap.get(Servo.class,"intakeRight");
        mWrist = hardwareMap.get(Servo.class,"wrist");
        mGrab = hardwareMap.get(Servo.class,"grab");
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRight = hardwareMap.get(Servo.class,"clawRight");
        slideLock = hardwareMap.get(Servo.class,"slideLock");
        mIntakeLeft.setDirection(Servo.Direction.REVERSE);

        mTouchSensor = hardwareMap.get(TouchSensor.class,"touch");

//        color = hardwareMap.get(ColorSensor.class,"color");
//
        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mGrab.setDirection(Servo.Direction.REVERSE);
//        mIntakeLeft.setDirection(Servo.Direction.REVERSE);


        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentArmMode = DcMotor.RunMode.RUN_USING_ENCODER;
        currentSlideMode = DcMotor.RunMode.RUN_USING_ENCODER;

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

        currentArmPower = mArm.getPower();
        currentSlideLeftPower = mSlideLeft.getPower();
        currentSlideRightPower = mSlideRight.getPower();
        currentArmPos = mArm.getCurrentPosition();
        currentSlideLeftPos = mSlideLeft.getCurrentPosition();
        currentSlideRightPos = mSlideRight.getCurrentPosition();
        currentTouchSensorState = mTouchSensor.isPressed();

        if(currentSlideMode == DcMotor.RunMode.RUN_TO_POSITION){
            if(Action.getCurrentActionType() != "SlideAction"){ //Math.abs(getSlideError())<10
                if(armTargetPosition != SSValues.ARM_DOWN && slideTargetPosition != SSValues.SLIDE_MIN){
                    setSlidePowerWrapper(0.3);
                }else if(Math.abs(getSlideError())<10){
                    setSlidePowerWrapper(0);
                }
            }
        }

//        setArmPowerWrapper(armPidCtrl.update(currentArmPos - armTargetPosition));

//        if(armTargetPosition == SSValues.ARM_DOWN && mArm.getPower() > 0.1){
//            mArm.setPower(-1);
////            mArm.setPower(Math.max(ArmAdjustment.armMinPower, Math.min(ArmAdjustment.coefficient*Math.cos(mArm.getCurrentPosition()*Math.PI/2200),1)));
//        }
//        if((armTargetPosition - mArm.getCurrentPosition() < 0 && Math.abs(getArmTargetPosition() - getArmPosition())<50)){
//            if (armTargetPosition == SSValues.ARM_UP && getArmTargetPosition() - getArmPosition() < 0) {
//                mArm.setPower(-0.3);
//            }else{
//                mArm.setPower(0);
//            }
//        }

        if(Math.abs(armTargetPosition - currentArmPos)<20){
            setArmPowerWrapper(0);
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
        CUSTOM_INTAKE,
        LOW_BASKET,
        HIGH_CHAMBER,
        ASCENT,
        INTAKE_SPECIMEN,
        HIGH_CHAMBER_AIM
        //Etc.
    }

    ///////////////////////////////////////ARM//////////////////////////////////////////////////////
    private int armTargetPosition = 0;
    private int armError;
    public void setArmPosition(int pos, double power){
        setArmPowerWrapper(power);
        armTargetPosition = pos;
        armError = getArmPosition() - armTargetPosition;
        if(armError>0){
            armPidCtrl.setOutputBounds(-1,1);
        }else{
            armPidCtrl.setOutputBounds(-1,1);
        }
        setArmModeWrapper(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmByP(int pos, double power){
        armTargetPosition = pos;
        mArm.setTargetPosition(armTargetPosition);
        setArmModeWrapper(DcMotor.RunMode.RUN_TO_POSITION);
        setArmPowerWrapper(power);
    }

    public void resetArmEncoder(){
        setArmModeWrapper(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmModeWrapper(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmByPower(int pos,double power){
        armTargetPosition = pos;
        setArmModeWrapper(DcMotor.RunMode.RUN_USING_ENCODER);
        setArmPowerWrapper(power);
    }

    public void setArmPower(double power){
        setArmPowerWrapper(power);
    }

    ////////////////////////////////////////SLIDES//////////////////////////////////////////////////
    private int slideTargetPosition = 0;
    public void setSlidePosition(int pos, double power) {
        slideTargetPosition = pos;
        setSlideModeWrapper(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(slideTargetPosition < getSlidesPosition() && Math.abs(getArmPosition() - SSValues.ARM_UP) < 20){
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
        slideTargetPosition = pos;
        mSlideRight.setTargetPosition(pos);
        mSlideLeft.setTargetPosition(pos);
        setSlideModeWrapper(DcMotor.RunMode.RUN_TO_POSITION);
        setSlidePowerWrapper(power);
    }

    public void resetSlide(){
        slideTargetPosition = SSValues.SLIDE_MIN;
        setSlideModeWrapper(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setSlidePowerWrapper(-0.3);

        opMode.sleep(50);
        setSlidePowerWrapper(0);
        setSlideModeWrapper(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetSlideEncoder(){
        slideTargetPosition = SSValues.SLIDE_MIN;
        setSlideModeWrapper(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setSlidesByPower(int pos, double power) {
        slideTargetPosition = pos;
        setSlideModeWrapper(DcMotor.RunMode.RUN_USING_ENCODER);

//            rSlidePidCtrl.setOutputBounds(-0, 0);
//            lSlidePidCtrl.setOutputBounds(-0, 0);
        setSlidePowerWrapper(power);
    }

    public void setSlidePower(double power){
        setSlidePowerWrapper(power);
    }

    public void lockSlide(){
        slideLock.setPosition(SSValues.SLIDE_LOCK_LOCKED_TIGHT);
        opMode.sleep(50);
        slideLock.setPosition(SSValues.SLIDE_LOCK_LOCKED_NORMAL);
    }
    public void unlockSlide(){
        slideLock.setPosition(SSValues.SLIDE_LOCK_DEFAULT);
    }
    public double getSlideLockPosition(){
        return slideLock.getPosition();
    }


//    public void stopIntake(){
//        controlRight.setStatus(false);
//        controlLeft.setStatus(false);
//    }
//    public void startIntake(){
//        controlRight.setStatus(true);
//        controlLeft.setStatus(true);
//    }

//    public void runFor(int ms){
//        long startTime = System.currentTimeMillis();
//        boolean stopped = false;
//        controlRight.setStatus(true);
//        controlLeft.setStatus(true);
//        if(startTime+ms < System.currentTimeMillis() && !stopped){
//            controlRight.setStatus(false);
//            controlLeft.setStatus(false);
//            stopped = true;
//        }
//    }

    public void setIntake(double val){
        mIntakeLeft.setPosition(val);
        mIntakeRight.setPosition(val);
    }
    public void setWristPos(double pos){
        if(currentWristPos != pos){
            currentWristPos = pos;
            mWrist.setPosition(pos);
        }
    }
    public void setGrabPos(double pos){
        if(currentGrabPos != pos){
            currentGrabPos = pos;
            mGrab.setPosition(pos);
        }
    }
    public double getGrabPos(){
        return currentGrabPos;
//        return mGrab.getPosition();
    }
    public void setClawLeftPos(double pos){clawLeft.setPosition(pos);}
    public void setClawRightPos(double pos){clawRight.setPosition(pos);}


    ///////////////////////////////////GETTERS AND SETTERS//////////////////////////////////////////
    public int getArmPosition(){
        return currentArmPos;
    }

    public int getSlideRightPosition(){
        return currentSlideRightPos;
    }
    public int getSlideLeftPosition(){
        return currentSlideLeftPos;
    }

    public DcMotor.RunMode getSlideMode(){
        return currentSlideMode;
    }
    public int getSlidesPosition(){
        return (currentSlideLeftPos+currentSlideRightPos)/2;
    }
    public double getWristPosition(){
        return currentWristPos;
    }
    public double getArmPower(){
        return currentArmPower;
    }
    public int getArmTargetPosition(){
        return armTargetPosition;
    }
    public int getSlideTargetPosition(){
        return slideTargetPosition;
    }
    public double getSlidePower(){
        return currentSlideLeftPower;
    }
    public double getSlideVelocity(){return mSlideLeft.getVelocity();}
    public double getSlideError(){
        return (double) (slideTargetPosition - (currentSlideLeftPos + currentSlideRightPos)/2);
    }
    public boolean getTouchSensorPressed(){
        return currentTouchSensorState;
    }
    public List<Integer> getColorRGBAValues(int threshold) {
//        if (cachedColor.get(3)==-1){
//            cachedColor.clear();
//            cachedColor.add(0,color.red());
//            cachedColor.add(1,color.green());
//            cachedColor.add(2,color.blue());
//            cachedColor.add(3,color.alpha());
//            return cachedColor;
//        }else{
//            int a=color.alpha();
//            if (Math.abs(cachedColor.get(3) - a) > threshold) {
//                cachedColor.clear();
//                cachedColor.add(0, color.red());
//                cachedColor.add(1, color.green());
//                cachedColor.add(2, color.blue());
//                cachedColor.add(3, a);
//            }
            return cachedColor;
//        }
    }

    public boolean colorSensorCovered(){
//        return color.alpha() > 90;
        return false;
//        List<Integer> rgbaValues = getColorRGBAValues();
//        return Collections.max(rgbaValues)>90;
    }
    public String colorOfSample(){
        if(colorSensorCovered()){
            List<Integer> rgbaValues = getColorRGBAValues(10);//color should not change...?
            if(colorSensorCovered()){
                int r=rgbaValues.indexOf(Collections.max(rgbaValues));
                switch (r){
                    case 0:
                        return "red";
                    case 1:
                        return "yellow";
                    case 2:
                        return "blue";
                }
            }
            return "unknown";
        }
        return "No sample detected";
    }

    public double getClawLeft(){return clawLeft.getPosition();}
    public double getClawRight(){return clawRight.getPosition();}
    public Sequences getSequence(){return sequence;}
    public Sequences getPreviousSequence(){return previousSequence;}
//    public boolean getSlideVelocityToZero(){
//        return slideZeroVelocity.toTrue();
//    }

    //WRAPPER FUNCTIONS, POSSIBLY VERY IMPORTANT FOR PROPER LOOP TIMES
    private void setArmModeWrapper(DcMotor.RunMode mode){
        if(currentArmMode != mode){
            mArm.setMode(mode);
            currentArmMode = mode;
        }
    }
    private void setSlideModeWrapper(DcMotor.RunMode mode){
        if(currentSlideMode != mode){
            mSlideRight.setMode(mode);
            mSlideLeft.setMode(mode);
            currentSlideMode = mode;
        }
    }

    private void setArmPowerWrapper(double power){
        if(currentArmPower != power){
            mArm.setPower(power);
            currentArmPower = power;
        }
    }

    private void setSlidePowerWrapper(double power){
        if(currentSlideLeftPower != power){
            mSlideLeft.setPower(power);
            mSlideRight.setPower(power);
            currentSlideLeftPower = power;
            currentSlideRightPower = power;
        }
    }

}