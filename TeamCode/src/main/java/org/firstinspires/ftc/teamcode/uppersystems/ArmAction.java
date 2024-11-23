package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class ArmAction extends Action {
    private int toleranceRange = 200;
    private SuperStructure upper;
    //Params not in super class
    private int armTarget;
    private double power = 1;
    public static double armMinPower = 0.16;
    public static double armDownCoefficient = 1.8;

    public ArmAction(SuperStructure upper, int armTarget){
        this.upper = upper;
        this.armTarget = armTarget;
    }

    public ArmAction(SuperStructure upper, int armTarget, int toleranceRange){
        this.upper = upper;
        this.armTarget = armTarget;
        this.toleranceRange = toleranceRange;
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
        if((Math.abs(getError()) < 10)){
            return true;
        }else{
            return false;
        }
    }

    public void actuate() {
        upper.setArmByP(armTarget, power);
    }

    //Functions not in super class
    public void setArmTarget(int target) {
        target = this.armTarget;
    }

    public void forceStop(){
        upper.setArmByP(armTarget, 0);
        toleranceRange = 10000;
    }

    public String returnType(){
        return "ArmAction";
    }


//    public void setPower(double power) {
//        power = this.power;
//    }

}
