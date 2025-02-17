package org.firstinspires.ftc.teamcode.actions.actioncore;

import org.firstinspires.ftc.teamcode.SuperStructure;

import java.util.function.BooleanSupplier;

/** Just a 3-branched action.
 */
public class CancellableFinishConditionActionGroup extends Action {
    private int toleranceRange = 100;
    private SuperStructure upper;
    //Params not in super class
    private Action action;
    private BooleanSupplier finishCondition;
    private BooleanSupplier cancelCondition;
    private Runnable runOnFinish;
    private Runnable runOnActionEnd;
    private Runnable runOnCancel;
    private boolean forceStop = false;

    public CancellableFinishConditionActionGroup(Action action, BooleanSupplier finishCondition, BooleanSupplier cancelCondition, Runnable runOnFinish, Runnable runOnActionEnd, Runnable runOnCancel){
        this.action = action;
        this.finishCondition = finishCondition;
        this.runOnFinish = runOnFinish;
        this.runOnActionEnd = runOnActionEnd;
        this.cancelCondition = cancelCondition;
        this.runOnCancel = runOnCancel;
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
        if(cancelCondition.getAsBoolean()){
            runOnCancel.run();
        }
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
        return "CancellableFinishConditionActionGroup";
    }

}
