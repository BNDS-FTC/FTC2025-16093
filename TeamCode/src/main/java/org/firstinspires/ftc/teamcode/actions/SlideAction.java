package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.MotorAction;

@Config
public class SlideAction extends MotorAction {

    public SlideAction(SuperStructure upper, int target){
        super(upper, target);
    }

    public SlideAction(SuperStructure upper, int target, int toleranceRange){
        super(upper, target, toleranceRange);
    }

    public SlideAction(SuperStructure upper, int target, int toleranceRange, double power){
        super(upper, target, toleranceRange, power);
    }

    public int getError() {
        return target - upper.getSlidesPosition();
    }

    public void actuate() {
        upper.setSlidesByP(target,power);
    }

    public void stop(){
        upper.setSlidePower(0);
        toleranceRange = 10000;
        super.stop();
    }

    //Functions not in super class
    public void forceStop(){
        upper.setSlidePower(0);
        toleranceRange = 10000;
        finishRange = 10000;
//        Action.actions.remove(this);
    }

    public String toString() {
        return returnType() + " Target " + this.target + " Power " + this.power + " Error " + this.getError();
    }

    public String returnType(){
        return "SlideAction";
    }

}
