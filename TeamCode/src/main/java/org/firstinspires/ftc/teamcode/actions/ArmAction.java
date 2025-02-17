package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.MotorAction;

@Config
public class ArmAction extends MotorAction {
    private int armTarget;

    public ArmAction(SuperStructure upper, int target){
        super(upper, target-upper.armOffset);
        armTarget = target - upper.armOffset;
    }

    public ArmAction(SuperStructure upper, int target, int toleranceRange){
        super(upper, target-upper.armOffset, toleranceRange);
        armTarget = target - upper.armOffset;
    }

    public ArmAction(SuperStructure upper, int target, int toleranceRange, double power){
        super(upper, target-upper.armOffset, toleranceRange, power);
        armTarget = target - upper.armOffset;
    }

    public int getError() {
        return armTarget - upper.getArmPosition();
    }

    public void actuate() {
        upper.setArmByP(armTarget,power);
    }

    public void stop(){
        upper.setArmPower(0);
//        upper.armLimiter.reset(0);
        toleranceRange = 10000;
        super.stop();
    }

    //Functions not in super class
    public void forceStop(){
        upper.setArmPower(0);
//        upper.armLimiter.reset(0);
        toleranceRange = 10000;
        finishRange = 10000;
//        Action.actions.remove(this);
    }

    public String toString() {
        return returnType() + " Target " + this.armTarget + " Power " + this.power + " Error " + this.getError();
    }

    public String returnType(){
        return "ArmAction";
    }

}
