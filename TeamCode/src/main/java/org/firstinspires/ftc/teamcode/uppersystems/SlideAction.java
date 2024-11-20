package org.firstinspires.ftc.teamcode.uppersystems;

import org.firstinspires.ftc.teamcode.references.XCYBoolean;

public class SlideAction extends Action {
    private int toleranceRange = 30;
    private SuperStructure upper;
    //Params not in super class
    private int slideTarget;
    private double power = 1;

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
        return Math.abs(getError()) < toleranceRange || upper.getSlideVelocityToZero();
    }

    public void actuate() {
        upper.setSlidePosition(slideTarget, power);
    }
}
