package org.firstinspires.ftc.teamcode.uppersystems;

public class ArmAction extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private int armTarget;
    private double power = 0.6;

    public ArmAction(SuperStructure upper, int armTarget){
        this.upper = upper;
        this.armTarget = armTarget;
    }

    public int getError() {
        return armTarget - upper.getArmPosition();
    }

    public void setToleranceRange(int allowableError) {
        this.toleranceRange = allowableError;
    }


    public boolean isFinished(){
        if((Math.abs(getError()) < toleranceRange)){
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

    public void setPower(double power) {
        power = this.power;
    }

}
