package org.firstinspires.ftc.teamcode.uppersystems;

public class ParallelAction extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private int armTarget;
    private int slideTarget;
    private double armPower = 0.7;
    private double slidePower = 0.9;


    public ParallelAction(SuperStructure upper, int armTarget, int slideTarget){
        this.upper = upper;
        this.armTarget = armTarget;
        this.slideTarget = slideTarget;
    }

    public ParallelAction(SuperStructure upper, int armTarget, int slideTarget, int toleranceRange){
        this.upper = upper;
        this.armTarget = armTarget;
        this.slideTarget = slideTarget;
        this.toleranceRange = toleranceRange;
    }

    public ParallelAction(SuperStructure upper, int armTarget, int slideTarget, double armPower, double slidePower){
        this.upper = upper;
        this.armTarget = slideTarget;
        this.armPower = armPower;
        this.slidePower = slidePower;
    }

    public int getError() {
        return (armTarget - upper.getArmPosition() + (slideTarget - upper.getSlidesPosition()))/2;
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
        upper.setArmByP(armTarget, armPower);
        upper.setSlidesByP(slideTarget, slidePower);
    }

    public void forceStop(){
        upper.setSlidePosition(slideTarget, 0);
        toleranceRange = 100000;
    }

    public String returnType(){
        return "ParallelAction";
    }

    //Functions not in super class
    public void setArmTarget(int target) {
        target = this.armTarget;
    }

    public void setArmPower(double armPower) {
        armPower = this.armPower;
    }

}
