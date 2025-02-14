package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;

public class WristAction extends ServoAction {

    public WristAction(SuperStructure upper, double pos){
        super(upper,pos);
    }

    public WristAction(SuperStructure upper, double pos, int waitTime){
        super(upper,pos,waitTime);
    }

    public String returnType(){
        return "WristAction";
    }

    public void actuate() {
        upper.setWristPos(pos);
    }

}
