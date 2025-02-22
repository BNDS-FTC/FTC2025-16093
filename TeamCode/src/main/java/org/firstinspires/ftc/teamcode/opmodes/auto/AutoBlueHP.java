package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();

        VexpPushTwoBlueSamples();
//        intakeThreeBlueSamples();

        intakeSpecimenFromBlueWall(-6,0);
        blueChamberPlaceFromWall(12,0);

        intakeSpecimenFromBlueWall(-5,-0);
        blueChamberPlaceFromWall(10,-1);

        intakeSpecimenFromBlueWall(0,-1);
        blueChamberPlaceFromWall(8,-1);

        intakeSpecimenFromBlueWall(0,-1.3);
        blueChamberPlaceFromWall(6.5,-1);

        parkFromBlueChamber();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}