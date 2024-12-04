package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.references.Globals;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.testings.ArmAdjustment;

public class SlideSubsystem extends SubsystemBase {

    private final DcMotorEx mSlideLeft;
    private final DcMotorEx mSlideRight;

    public static PIDCoefficients rSlidePidConf = new PIDCoefficients(0.025, 0, 0);
    private final PIDFController rSlidePidCtrl;
    public static PIDCoefficients lSlidePidConf = new PIDCoefficients(0.025, 0, 0);
    private final PIDFController lSlidePidCtrl;
    public static PIDCoefficients rSlidePidConfVertical = new PIDCoefficients(0.008, 0, 0);
    private final PIDFController rSlidePidCtrlVertical;
    public static PIDCoefficients lSlidePidConfVertical = new PIDCoefficients(0.008, 0, 0);
    private final PIDFController lSlidePidCtrlVertical;

    public int slideMode;
    public int slideTargetPosition;

    public SlideSubsystem(HardwareMap hardwareMap) {
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rSlidePidCtrl = new PIDFController(rSlidePidConf);
        lSlidePidCtrl = new PIDFController(lSlidePidConf);
        rSlidePidCtrlVertical = new PIDFController(rSlidePidConfVertical);
        lSlidePidCtrlVertical = new PIDFController(lSlidePidConfVertical);
        slideMode = 0;

    }

//    @Override
//    public void periodic() {
//        // This method will be called once per scheduler run
//        if(Math.abs(getSlideError())<20){
//            if(slideTargetPosition == SSValues.SLIDE_MAX) {
//                mSlideLeft.setPower(0.3);
//                mSlideRight.setPower(0.3);
//            } else {
//                mSlideLeft.setPower(0.1);
//                mSlideRight.setPower(0.1);
//            }
//        }
//    }

    public void setSlidePosition(int pos, double power) {
        slideTargetPosition = pos;
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(slideTargetPosition < mSlideLeft.getCurrentPosition()){
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

        //THIS IS MISSING A SLEEP!!!!!!!!!

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
        rSlidePidCtrl.setOutputBounds(-0, 0);
        lSlidePidCtrl.setOutputBounds(-0, 0);
        mSlideRight.setPower(power);
        mSlideLeft.setPower(power);
    }

    public void setSlidePower(double power){
        mSlideLeft.setPower(power);
        mSlideRight.setPower(power);
    }

    public void setToCustomSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMode = 1;
    }
    public void setToRegularSlide(){
        slideMode = 0;
    }
    public int getSlideMode(){return slideMode;}

    public int getSlideRightPosition(){
        return mSlideRight.getCurrentPosition();
    }
    public int getSlideLeftPosition(){
        return mSlideLeft.getCurrentPosition();
    }
    public int getSlidesPosition(){
        return (mSlideLeft.getCurrentPosition()+mSlideRight.getCurrentPosition())/2;
    }
    public int getSlideTargetPosition(){
        return slideTargetPosition;
    }
    public double getSlidePower(){
        return mSlideLeft.getPower();
    }
    public double getSlideVelocity(){return mSlideLeft.getVelocity();}
    public double getSlideError(){
        return (double) slideTargetPosition - (mSlideLeft.getCurrentPosition() + slideTargetPosition)/2;
    }
}