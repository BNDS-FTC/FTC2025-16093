package org.firstinspires.ftc.teamcode.actions.actioncore;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

public class DriveAction extends Action {
    //Params not in super class
    protected NewMecanumDrive drive;
    protected Pose2d targetPose;
    protected int correctTime;
    private int waitTime = 0;
    protected Runnable runWhileMoving;
    protected long timeOnStart;

    public DriveAction(NewMecanumDrive drive, Pose2d targetPose, int correctTime, int waitTime){
        this.drive = drive;
        this.targetPose = targetPose;
        this.correctTime = correctTime;
        this.waitTime = waitTime;
        this.runWhileMoving = ()->{};
        timeOnStart = System.currentTimeMillis();
    }

    public DriveAction(NewMecanumDrive drive, Pose2d targetPose, int correctTime, int waitTime, Runnable runWhileMoving){
        this.drive = drive;
        this.targetPose = targetPose;
        this.correctTime = correctTime;
        this.runWhileMoving = runWhileMoving;
        this.waitTime = waitTime;
        timeOnStart = System.currentTimeMillis();
    }


    public boolean canStartNext(){
        if(System.currentTimeMillis() - timeOnStart > waitTime){
            return true;
        }else{
            return false;
        }
    }

    public boolean isFinished(){
        return !drive.simpleMoveIsActivate;
    }

    public void actuate() {
        drive.moveTo(targetPose,correctTime,runWhileMoving);
    }

    public void stop(){
        drive.setMotorPowers(0,0,0,0);
        drive.simpleMoveIsActivate = false;
    }

    //Functions not in super class
    public void forceStop(){
        drive.setMotorPowers(0,0,0,0);
        drive.simpleMoveIsActivate = false;
    }

    public String toString() {
        return returnType() + " Target " + this.targetPose + " Correct Time " + this.correctTime + " Wait Time " + this.waitTime;
    }

    public String returnType(){
        return "MotorAction";
    }



}
