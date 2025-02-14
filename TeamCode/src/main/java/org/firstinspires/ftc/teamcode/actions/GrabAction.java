package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;

public class GrabAction extends ServoAction {

    public GrabAction(SuperStructure upper, double pos){
        super(upper,pos);
    }

    public GrabAction(SuperStructure upper, double pos, int waitTime){
        super(upper,pos,waitTime);
    }

    public String returnType(){
        return "GrabAction";
    }

    public void actuate() {
        upper.setGrabPos(pos);
    }

}
