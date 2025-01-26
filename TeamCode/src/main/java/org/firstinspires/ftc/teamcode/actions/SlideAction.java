package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;

public class SlideAction extends Action {
    private int toleranceRange = 200;
    private int finishRange = 30;
    private SuperStructure upper;
    private long timeOnStart;
    //Params not in super class
    private int slideTarget;
    private double power = 1;

    public SlideAction(SuperStructure upper, int slideTarget){
        this.upper = upper;
        this.slideTarget = slideTarget;
        timeOnStart = System.currentTimeMillis();
    }

    public SlideAction(SuperStructure upper, int slideTarget, int toleranceRange){
        this.upper = upper;
        this.slideTarget = slideTarget;
        this.toleranceRange = toleranceRange;
        timeOnStart = System.currentTimeMillis();
    }

    public SlideAction(SuperStructure upper, int slideTarget, int toleranceRange, double power){
        this.upper = upper;
        this.slideTarget = slideTarget;
        this.toleranceRange = toleranceRange;
        this.power = power;
    }
    public int getError() {
        return slideTarget - upper.getSlidesPosition();
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

    public void stop(){
        upper.setSlidePower(0);
        toleranceRange = 100000;
//        super.stop();
    }

    public void forceStop(){
        upper.setSlidePower(0);
        toleranceRange = 100000;
        finishRange = 100000;
        Action.actions.remove(this);
    }

    public String returnType(){
        return "SlideAction";
    }


    public String toString() {
        return returnType() + " Target " + this.slideTarget + " Power " + this.power + " Error " + this.getError();
    }

    public void actuate() {
        upper.setSlidesByP(slideTarget, power);
    }
}
