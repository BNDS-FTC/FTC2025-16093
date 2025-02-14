package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.Action;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;

public class AscentAction extends Action {
    private SuperStructure.AscentState state;
    private SuperStructure upper;
    private int waitTime;

    public AscentAction(SuperStructure upper, SuperStructure.AscentState state){
        this.upper = upper;
        this.state = state;
        timeOnStart = System.currentTimeMillis();
    }

    public AscentAction(SuperStructure upper, SuperStructure.AscentState state, int waitTime){
        this.upper = upper;
        this.state = state;
        timeOnStart = System.currentTimeMillis();
        this.waitTime = waitTime;
    }

    public boolean canStartNext(){
        if(System.currentTimeMillis() - timeOnStart > waitTime){
            return true;
        }else{
            return false;
        }
    }

    public String returnType(){
        return "AscentAction";
    }

    public String toString() {
        return returnType() + " State " + this.state;
    }

    public void actuate() {
        upper.setAscentState(state);
    }

}
