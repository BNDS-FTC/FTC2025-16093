package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmAction extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private int armTarget;
    private double power = 1;
    public static double armMinPower = 0.65;
    public static double armDownCoeffiecient = 1.65;

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
    


    public boolean isFinished(){
        if((Math.abs(getError()) < toleranceRange)){
            return true;
        }else{
            return false;
        }
    }

    public void actuate() {
        if(armTarget > upper.getArmPosition()){
            upper.setArmByP(armTarget, 1);
        }else{
            upper.setArmByP(armTarget, Math.max(armMinPower, Math.min(armDownCoeffiecient*Math.cos(upper.getArmPosition()*Math.PI/2000),1)));
        }
    }

    //Functions not in super class
    public void setArmTarget(int target) {
        target = this.armTarget;
    }

//    public void setPower(double power) {
//        power = this.power;
//    }

}
