package org.firstinspires.ftc.teamcode.uppersystems;

public class SlideAction extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private int slideTarget;
    private double power = 0.7;

    public SlideAction(SuperStructure upper, int slideTarget){
        this.upper = upper;
        this.slideTarget = slideTarget;
    }

    public int getError() {
        return slideTarget - upper.getSlidePosition();
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
        upper.setSlidesByP(slideTarget, power);
    }

    //Functions not in super class
    public void setSlideTarget(int target) {
        target = this.slideTarget;
    }

    public void setPower(double power) {
        power = this.power;
    }

}
