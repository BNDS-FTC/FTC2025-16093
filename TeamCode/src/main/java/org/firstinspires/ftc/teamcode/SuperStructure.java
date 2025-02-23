package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.references.SSValues;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * 希望它不要爆掉...
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
    private final DcMotorEx armUp;
    private final DcMotorEx armDown;
    private final DcMotorEx slideRight;
    private final DcMotorEx slideLeft;

    private final Servo intakeLeft; // continuous
    private final Servo intakeRight;// continuous
    private final Servo wrist;
    private final Servo grab;
    private final Servo tail;
    private final Servo ascentLeft, ascentRight; //Continuous!



    Sequences sequence;
    Sequences previousSequence;

    public final TouchSensor touchSensor;

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

    private final ColorSensor color;
    private final DistanceSensor distance;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;
//    private XCYBoolean slideZeroVelocity;

    public static int armOffset;
    double currentArmPowerUp, currentArmPowerDown, currentSlideLeftPower, currentSlideRightPower;
    double currentWristPos, currentGrabPos, currentIntakePos, currentTailPos;
    int currentArmPosUp, currentArmPosDown, currentSlideLeftPos, currentSlideRightPos;
    boolean currentTouchSensorState = true;
    DcMotor.RunMode currentArmMode, currentSlideMode;
//    public SlewRateLimiter armLimiter;
//    private final ColorIdentification colorCalculator = new ColorIdentification(0.5f,1f,1f);

//    private final ServoPWMControl ascentLeftController,ascentRightController;

    public boolean slideTooHigh = false;
    private int count = 0;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure(LinearOpMode opMode, Runnable updateRunnable, int armOffset){
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.updateRunnable = updateRunnable;
        armPidCtrl = new PIDFController(armPidConf);
        rSlidePidCtrl = new PIDFController(rSlidePidConf);
        lSlidePidCtrl = new PIDFController(lSlidePidConf);
        rSlidePidCtrlVertical = new PIDFController(rSlidePidConfVertical);
        lSlidePidCtrlVertical = new PIDFController(lSlidePidConfVertical);

        armUp = hardwareMap.get(DcMotorEx.class,"armUp");
        armDown = hardwareMap.get(DcMotorEx.class,"armDown");

        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        mArmUp.setDirection(DcMotorSimple.Direction.REVERSE);
//        mArmDown.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        intakeRight = hardwareMap.get(Servo.class,"intakeRight");
        wrist = hardwareMap.get(Servo.class,"wrist");
        grab = hardwareMap.get(Servo.class,"grab");
        tail = hardwareMap.get(Servo.class, "tail");
        ascentLeft = hardwareMap.get(Servo.class, "ascentLeft");
        ascentRight = hardwareMap.get(Servo.class, "ascentRight");

//        ascentLeftController = new ServoPWMControl(mAscentLeft);
//        ascentRightController = new ServoPWMControl(mAscentRight);
//        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
//        clawRight = hardwareMap.get(Servo.class,"clawRight");
        intakeLeft.setDirection(Servo.Direction.REVERSE);
        tail.setDirection(Servo.Direction.REVERSE);


        touchSensor = hardwareMap.get(TouchSensor.class,"touch");

        color = hardwareMap.get(ColorSensor.class,"color");
//
        armUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //mGrab.setDirection(Servo.Direction.REVERSE);
//        mIntakeLeft.setDirection(Servo.Direction.REVERSE);


//        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mArmUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mArmDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRight.setTargetPosition(slideRight.getCurrentPosition());
        slideLeft.setTargetPosition(slideLeft.getCurrentPosition());
        armUp.setTargetPosition(armUp.getCurrentPosition());
        armDown.setTargetPosition(armDown.getCurrentPosition());


        currentArmMode = DcMotor.RunMode.RUN_USING_ENCODER;
        currentSlideMode = DcMotor.RunMode.RUN_USING_ENCODER;

//        armLimiter = new SlewRateLimiter(0.35);

        this.sequence = Sequences.RUN;
        this.previousSequence = Sequences.RUN;
        this.armOffset = armOffset;
        distance = hardwareMap.get(DistanceSensor.class, "color");
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
        count++;
        currentArmPowerUp = armUp.getPower();
        currentArmPowerDown = armDown.getPower();
        currentSlideLeftPower = slideLeft.getPower();
        currentSlideRightPower = slideRight.getPower();
        currentArmPosUp = armUp.getCurrentPosition();
        currentArmPosDown = armDown.getCurrentPosition();
        currentSlideLeftPos = slideLeft.getCurrentPosition();
        currentSlideRightPos = slideRight.getCurrentPosition();
//        RobotLog.d("SlideLeft: "+currentSlideLeftPos+" +SlideRight: "+currentSlideRightPos+" on cycle "+count);
        currentTouchSensorState = touchSensor.isPressed();
//        if((sequence == Sequences.INTAKE_FAR || sequence == Sequences.INTAKE_NEAR) && currentWristPos == SSValues.WRIST_INTAKE){
////            color.enableLed(true);
//            currentAlpha = color.alpha();
//            currentDistance = distance.getDistance(DistanceUnit.CM);
//        }
        slideTooHigh = armTargetPosition == SSValues.ARM_UP-armOffset && Math.abs(getArmError()) < 100;



        if(currentSlideMode == DcMotor.RunMode.RUN_TO_POSITION){
            if(Math.abs(getSlideError())<8){
                if(armTargetPosition == (SSValues.ARM_UP-armOffset) && slideTargetPosition != SSValues.SLIDE_MIN){
                    setSlidePowerWrapper(0.3);
                }else{
                    setSlidePowerWrapper(0);
                }
            }
        }

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


        if(currentArmMode == DcMotor.RunMode.RUN_TO_POSITION){
            if(Math.abs(getArmTargetPosition() - getArmPosition())<13){
                setArmPowerWrapper(0);
            }
//            else {
//                if (getArmPosition() > (SSValues.ARM_UP * 0.6) && getArmTargetPosition() == SSValues.ARM_UP) {
//                    setArmPowerWrapper(0.4);
//                } else if (getArmPosition() > (SSValues.ARM_UP * 0.2) && getArmTargetPosition() == SSValues.ARM_UP) {
//                    setArmPowerWrapper(0.7);
//                } else {
//                    setArmPowerWrapper(armLimiter.calculate(currentArmPowerUp));
//                }
//            }
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
        LOW_BASKET,
        HIGH_CHAMBER_PLACE,
        INTAKE_SPECIMEN,
        HIGH_CHAMBER_AIM,
        ASCENT
        //Etc.
    }

    public enum AscentState {
        ASCENT_UP,
        ASCENT_DOWN,
        ASCENT_DOWN_A_LITTLE,
        ASCENT_DOWN_SOME_MORE
    }

    ///////////////////////////////////////ARM//////////////////////////////////////////////////////
    private int armTargetPosition = 0;
    public void setArmTargetPosition(int pos){
        armTargetPosition = pos; //BE REALLY REALLY CAREFUL WITH THIS OKAY????
    }
    private int armError;
    public void setArmPosition(int pos, double power){
        setArmPowerWrapper(power);
        armTargetPosition = pos;
        armError = getArmPosition() - armTargetPosition;
        armPidCtrl.setOutputBounds(-1,1);
        setArmModeWrapper(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmByP(int pos, double power){
        armTargetPosition = pos;
        armUp.setTargetPosition(pos);
        armDown.setTargetPosition(pos);
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
    public int slideTargetPosition = 0;
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
        slideRight.setTargetPosition(pos);
        slideLeft.setTargetPosition(pos);
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
    }
    public void unlockSlide(){

    }
    public double getSlideLockPosition(){
        return -1;
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
        intakeLeft.setPosition(val);
        intakeRight.setPosition(val);
        currentIntakePos = val;
    }
    public void setWristPos(double pos){
        currentWristPos = pos;
        wrist.setPosition(pos);
    }
    public void setTailPos(double pos){
        if(currentTailPos != pos){
            tail.setPosition(pos);
            currentTailPos = pos;
        }
    }

    public double getTailPos(){
        return currentTailPos;
    }


    public void setAscentState(AscentState state){
        if(state == AscentState.ASCENT_UP){
            ascentLeft.setPosition(SSValues.ASCENT_LEFT_UP);
            ascentRight.setPosition(SSValues.ASCENT_RIGHT_UP);
        }else if(state == AscentState.ASCENT_DOWN_A_LITTLE){
            ascentLeft.setPosition(SSValues.ASCENT_LEFT_DOWN_A_LITTLE);
            ascentRight.setPosition(SSValues.ASCENT_RIGHT_DOWN_A_LITTLE);
        }else if(state == AscentState.ASCENT_DOWN_SOME_MORE){
            ascentLeft.setPosition(SSValues.ASCENT_LEFT_DOWN_SOME_MORE);
            ascentRight.setPosition(SSValues.ASCENT_RIGHT_DOWN_SOME_MORE);
        }else if(state == AscentState.ASCENT_DOWN) {
            ascentLeft.setPosition(SSValues.ASCENT_LEFT_DOWN);
            ascentRight.setPosition(SSValues.ASCENT_RIGHT_DOWN);
        }
    }

    public void setGrabPos(double pos){
        currentGrabPos = pos;
        grab.setPosition(pos);
    }
    public double getGrabPos(){
        return currentGrabPos;
    }


    ///////////////////////////////////GETTERS AND SETTERS//////////////////////////////////////////
    public int getArmPosition(){
        return (currentArmPosDown+currentArmPosUp)/2;
    }
    public int getArmPositionUp(){
        return currentArmPosUp;
    }
    public int getArmPositionDown(){
        return currentArmPosDown;
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
        return (currentArmPowerUp+currentArmPowerDown)/2;
    }
    public int getArmTargetPosition(){
        return armTargetPosition;
    }
    public int getSlideTargetPosition(){
        return slideTargetPosition;
    }
    public double getSlidePower(){
        return (currentSlideLeftPower+currentSlideRightPower)/2;
    }
    public double getSlideLeftVelocity(){return slideLeft.getVelocity();}
    public double getSlideRightVelocity(){return slideRight.getVelocity();}

    public double getSlideError(){
        return (double) (slideTargetPosition - (currentSlideLeftPos + currentSlideRightPos)/2);
    }
    public boolean getTouchSensorPressed(){
        return currentTouchSensorState;
    }

    public double getArmError(){
        return (double) (armTargetPosition - (currentArmPosUp+currentArmPosDown)/2);
    }

    public double getCurrentIntakePosition(){
        return currentIntakePos;
    }
//    public NormalizedRGBA getColorRGBAValues() {
//        return color.getNormalizedColors();
//    }
    public double getDistance() {
        return distance.getDistance(DistanceUnit.MM);
    }


    public boolean colorSensorCovered(){
        return color.alpha() > 30 && getDistance() < 49;
//        List<Integer> rgbaValues = getColorRGBAValues();
//        return Collections.max(rgbaValues)>90;
    }

//    public double getColorError() {
//        return colorCalculator.error;
//    }

    private int redThreshold = 35;
    private int yellowThreshold = 35;
    private int blueThreshold = 30;
    private int indexOfMaxRGB = -1;
    private int currentAlpha = -1;
    private int currentRed = -1;
    private int currentGreen = -1;
    private int currentBlue = -1;
    private double currentDistance = 100000;

    List<Integer> rgbaValues;
    private final List<Integer> cachedColor = new ArrayList<>(Arrays.asList(0,0,0,-1));



    public List<Integer> getColorRGBAValues(int threshold) {
        if (cachedColor.get(3)==-1){
            cachedColor.clear();
            cachedColor.add(0,color.red());
            cachedColor.add(1,color.green());
            cachedColor.add(2,color.blue());
            cachedColor.add(3,color.alpha());
            return cachedColor;
        }else{
            int a=color.alpha();
            if (Math.abs(cachedColor.get(3) - a) > threshold) {
                cachedColor.clear();
                cachedColor.add(0, color.red());
                cachedColor.add(1, color.green());
                cachedColor.add(2, color.blue());
                cachedColor.add(3, a);
            }
            return cachedColor;
        }
    }



    public int alphaAdjustedSampleColor(){
        rgbaValues = getColorRGBAValues(5);//color should not change...?
        if(colorSensorCovered()) {
            indexOfMaxRGB = rgbaValues.indexOf(Collections.max(rgbaValues));
            currentRed = rgbaValues.get(0);
            currentGreen = rgbaValues.get(1);
            currentBlue = rgbaValues.get(2);
//            currentAlpha = rgbaValues.get(3);
            if (indexOfMaxRGB == 0) {
                return 0;
            }else if (indexOfMaxRGB == 1 && compareColorDiff(currentGreen, currentRed, currentBlue)) {
                return 1;
            } else if (indexOfMaxRGB == 2) {
                return 2;
            }else{
                return -1;
            }
        }
        return -1;
    }


    private int findIndexOfMax(int[] arr){
        int max = -1;
        for(int i:arr){
            if(i > max){
                max = i;
            }
        }
        return max;
    }


    public int colorOfSample(){
        return alphaAdjustedSampleColor();
    }

    private boolean compareColorDiff(int target, int closeTo, int farFrom){
        if(Math.abs(target-closeTo)< Math.abs(target-farFrom)){
            return true;
        }
        return false;
    }


//    public double getClawLeft(){return clawLeft.getPosition();}
//    public double getClawRight(){return clawRight.getPosition();}
    public Sequences getSequence(){return sequence;}
    public Sequences getPreviousSequence(){return previousSequence;}
//    public boolean getSlideVelocityToZero(){
//        return slideZeroVelocity.toTrue();
//    }

    //WRAPPER FUNCTIONS, POSSIBLY VERY IMPORTANT FOR PROPER LOOP TIMES
    private void setArmModeWrapper(DcMotor.RunMode mode){
        if(currentArmMode != mode){
            armUp.setMode(mode);
            armDown.setMode(mode);
            currentArmMode = mode;
        }
    }
    private void setSlideModeWrapper(DcMotor.RunMode mode){
        if(currentSlideMode != mode){
            slideRight.setMode(mode);
            slideLeft.setMode(mode);
            currentSlideMode = mode;
        }
    }

    private void setArmPowerWrapper(double power){
        if(currentArmPowerUp != power){
            armUp.setPower(power);
            armDown.setPower(power);
            currentArmPowerUp = power;
        }
    }

    private void setSlidePowerWrapper(double power){
        if(currentSlideLeftPower != power){
            slideLeft.setPower(power);
            slideRight.setPower(power);
            currentSlideLeftPower = power;
            currentSlideRightPower = power;
        }
    }


}