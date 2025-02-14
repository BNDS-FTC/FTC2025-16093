package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;

public class IntakeAction extends ServoAction {

    public IntakeAction(SuperStructure upper, double pos){
        super(upper,pos);
    }

    public IntakeAction(SuperStructure upper, double pos, int waitTime){
        super(upper,pos,waitTime);
    }

    public String returnType(){
        return "IntakeAction";
    }

    public void actuate() {
        upper.setIntake(pos);
    }

}
