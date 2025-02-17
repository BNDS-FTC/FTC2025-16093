package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class NewAutoBlueHP extends AutoMaster{
    //TODO: Make a version w/o color sensor
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();

        VexpPushTwoBlueSamples();
//        intakeThreeBlueSamples();

        intakeSpecimenFromWall(-3,0);
        blueChamberPlaceFromWall(12,0);

        intakeSpecimenFromWall(0,-0);
        blueChamberPlaceFromWall(10,-1);

        intakeSpecimenFromWall(0,-1);
        blueChamberPlaceFromWall(8,-1);

        intakeSpecimenFromWall(0,-1);
        blueChamberPlaceFromWall(5,-1);

        intakeSpecimenFromWall(0,-1);

        while(opModeIsActive()){
            super.update.run();
        }

    }


}