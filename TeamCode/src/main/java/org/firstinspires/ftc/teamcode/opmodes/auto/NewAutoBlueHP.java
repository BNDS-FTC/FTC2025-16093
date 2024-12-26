package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class NewAutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();

//        highChamberPlace();
//
//        expPushTwoBlueSamples();
//
//        expPrepareForClawBlueSampleUp(0,0,0);
//        expClawBlueSampleUp(0,0);
//
//        VexpMoveToBlueChamberPlace(3);
//        highChamberPlace();
//
//        expPrepareForClawBlueSampleUp(17,-0.5,100);
//        expClawBlueSampleUp(17,-0.5);
//
//        VexpMoveToBlueChamberPlace(6);
//        highChamberPlace();
//
//        expPrepareForClawBlueSampleUp(17,0.3,0);
//        expClawBlueSampleUp(17,0.3);
//
//        VexpMoveToBlueChamberPlace(9);
//        highChamberPlace();
//        expPrepareForTeleOpBlue();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}
