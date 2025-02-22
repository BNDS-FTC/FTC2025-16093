package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRedHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(15,-62.3 ,Math.toRadians(-90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToRedChamberPlace();

        VexpPushTwoRedSamples();

        intakeSpecimenFromRedWall(4.7,-1.7);
        redChamberPlaceFromWall(-13,0);

        intakeSpecimenFromRedWall(0.5,-0);
        redChamberPlaceFromWall(-11,-1);

        intakeSpecimenFromRedWall(0.5,0);
        redChamberPlaceFromWall(-9,-1);

        intakeSpecimenFromRedWall(0.5,0);
        redChamberPlaceFromWall(-7,-1.5);

        parkFromRedChamber();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}