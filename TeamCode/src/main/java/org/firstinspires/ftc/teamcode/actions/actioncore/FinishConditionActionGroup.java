package org.firstinspires.ftc.teamcode.actions.actioncore;

import org.firstinspires.ftc.teamcode.SuperStructure;

import java.util.function.BooleanSupplier;

public class FinishConditionActionGroup extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private Action action;
    private BooleanSupplier finishCondition;
    private Runnable runOnFinish;
    private Runnable runOnActionEnd;
    private boolean forceStop = false;

    public FinishConditionActionGroup(Action action, BooleanSupplier finishCondition,Runnable runOnFinish, Runnable runOnActionEnd){
        this.action = action;
        this.finishCondition = finishCondition;
        this.runOnFinish = runOnFinish;
        this.runOnActionEnd = runOnActionEnd;
    }

    public int getError() {
        return action.getError();
    }

    public boolean canStartNext(){
        return action.canStartNext();
    }

    public boolean isFinished(){
        return finishCondition.getAsBoolean() || forceStop || action.isFinished();
    }

    public void actuate() {
        action.actuate();
    }

    public void stop(){
        action.stop();
        if(finishCondition.getAsBoolean()){
            runOnFinish.run();
        }else{
            runOnActionEnd.run();
        }
    }

    public void forceStop(){
        forceStop = true;
        action.forceStop();
    }

    public String toString() {
        return returnType() + " " + this.action.toString();
    }

    public String returnType(){
        return "FinishConditionActionGroup";
    }

}
