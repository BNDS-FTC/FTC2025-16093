package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class ArmAction extends Action {
    private int toleranceRange = 200;
    private int finishRange = 10;
    private SuperStructure upper;
    //Params not in super class
    private int armTarget;
    private double power = 1;
    private long timeOnStart;
    public static double armMinPower = 0.16;
    public static double armDownCoefficient = 1.8;

    public ArmAction(SuperStructure upper, int armTarget){
        this.upper = upper;
        this.armTarget = armTarget-upper.armOffset;
        timeOnStart = System.currentTimeMillis();
    }

    public ArmAction(SuperStructure upper, int armTarget, int toleranceRange){
        this.upper = upper;
        this.armTarget = armTarget-upper.armOffset;
        this.toleranceRange = toleranceRange;
        timeOnStart = System.currentTimeMillis();
    }

    public ArmAction(SuperStructure upper, int armTarget, int toleranceRange, double power){
        this.upper = upper;
        this.armTarget = armTarget-upper.armOffset;
        this.toleranceRange = toleranceRange;
        this.power = power;
        timeOnStart = System.currentTimeMillis();
    }

//    public ArmAction(SuperStructure upper, int slideTarget, double power){
//        this.upper = upper;
//        this.armTarget = slideTarget;
//        this.power = power;
//    }

    public int getError() {
        return armTarget - upper.getArmPosition();
    }

    public boolean canStartNext(){
        if((Math.abs(getError()) < toleranceRange)){
            return true;
        }else{
            return false;
        }
    }

    public boolean isFinished(){
        if((Math.abs(getError()) < finishRange)){
            return true;
        }else{
            return false;
        }
    }

    public void actuate() {
//        upper.setArmByP(armTarget,power);
        if(armTarget - upper.getArmPosition() < 0){
            upper.setArmByPower(armTarget,-power);
        }else{
            upper.setArmByP(armTarget,power);
        }
    }

    public void stop(){
        upper.setArmPower(0);
        toleranceRange = 10000;
    }

    //Functions not in super class
    public void forceStop(){
        upper.setArmPower(0);
//        toleranceRange = 10000;
//        finishRange = 10000;
        Action.actions.remove(this);
    }

    public String toString() {
        return returnType() + " Target " + this.armTarget + " Power " + this.power + " Error " + this.getError();
    }

    public String returnType(){
        return "ArmAction";
    }



//    public void setPower(double power) {
//        power = this.power;
//    }

}
