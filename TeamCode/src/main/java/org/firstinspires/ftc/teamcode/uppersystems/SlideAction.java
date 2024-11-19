package org.firstinspires.ftc.teamcode.uppersystems;

public class SlideAction extends Action {
    private int toleranceRange = 30;
    private SuperStructure upper;
    //Params not in super class
    private int slideTarget;
    private double power = 0.8;

    public SlideAction(SuperStructure upper, int slideTarget){
        this.upper = upper;
        this.slideTarget = slideTarget;
    }

    public SlideAction(SuperStructure upper, int slideTarget, int toleranceRange){
        this.upper = upper;
        this.slideTarget = slideTarget;
        this.toleranceRange = toleranceRange;
    }

    public SlideAction(SuperStructure upper, int slideTarget, double power){
        this.upper = upper;
        this.slideTarget = slideTarget;
        this.power = power;
    }

    public int getError() {
        return slideTarget - upper.getSlidesPosition();
    }

    public boolean isFinished(){
        return Math.abs(getError()) < toleranceRange;
    }

    public void actuate() {
        if(!isFinished()){
            if(getError()>0){
                upper.setSlidesByPower(power);
            }else{
                upper.setSlidesByPower(-power);
            }
        }else{
            upper.setSlidesByPower(0);
        }
    }

    //Functions not in super class
    public void setSlideTarget(int target) {
        target = this.slideTarget;
    }

    public void setPower(double power) {
        power = this.power;
    }

}
