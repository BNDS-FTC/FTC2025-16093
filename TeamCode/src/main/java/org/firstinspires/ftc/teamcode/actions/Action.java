package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;

import java.util.ArrayList;

public class Action {
    private int error;
    private SuperStructure upper;
    private long timeOnStart;
    public final static ArrayList<Action> actions = new ArrayList<>(6);
    public static boolean stopBuilding = false;


    public int getError(){
        return 0;
    }

    public void actuate(){

    }

    public boolean isFinished(){
        return false;
    }

    public boolean canStartNext(){
        return false;
    }


    public String getType() {
        return "Action";
    }

    public void forceStop(){
    }

    public void stop(){
//        actions.remove(this);
    }

    public String toString(){
        return "There are no Actions being called.";
    }

    static Action emptyAction = new Action();
    static Action currentAction = emptyAction;
    public static void buildSequence(Runnable runWhileBuilding){
        if(!actions.isEmpty()){
            for (int i=0;i < actions.size();i++) {
                currentAction = actions.get(i);
                currentAction.actuate(); // Execute current action

                while(!currentAction.canStartNext()){
                    runWhileBuilding.run();

                    if(stopBuilding){
                        currentAction.forceStop();
                        actions.clear();
                        stopBuilding = false;
                        break;
                    }

                    if(currentAction.isFinished()){ //|| System.currentTimeMillis() - currentAction.timeOnStart > 10000
                        currentAction.stop();
                    }
                }
            }
            actions.clear(); // Clear completed actions and reset mode
            currentAction = emptyAction;
        }
    }

    public static String showCurrentAction(){
        return currentAction.toString();
    }

    public static String getCurrentActionType(){
        return currentAction.getType();
    }
    public static void clearActions(){
        currentAction = emptyAction;
        Action.actions.clear();
    }

}
