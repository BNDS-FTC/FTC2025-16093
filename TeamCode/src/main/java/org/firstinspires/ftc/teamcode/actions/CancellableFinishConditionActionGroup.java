package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;

import java.util.function.BooleanSupplier;

public class CancellableFinishConditionActionGroup extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private Action action;
    private BooleanSupplier finishCondition;
    private BooleanSupplier cancelCondition;
    private Runnable runOnFinish;
    private Runnable runOnActionEnd;
    private boolean forceStop = false;

    public CancellableFinishConditionActionGroup(Action action, BooleanSupplier finishCondition, BooleanSupplier cancelCondition, Runnable runOnFinish, Runnable runOnActionEnd){
        this.action = action;
        this.finishCondition = finishCondition;
        this.runOnFinish = runOnFinish;
        this.runOnActionEnd = runOnActionEnd;
        this.cancelCondition = cancelCondition;
    }

    public int getError() {
        return action.getError();
    }

    public boolean canStartNext(){
        return action.canStartNext();
    }

    public boolean isFinished(){
        return finishCondition.getAsBoolean() || cancelCondition.getAsBoolean() || forceStop || action.isFinished();
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

    public String returnType(){
        return "FinishConditionActionGroup";
    }

}
