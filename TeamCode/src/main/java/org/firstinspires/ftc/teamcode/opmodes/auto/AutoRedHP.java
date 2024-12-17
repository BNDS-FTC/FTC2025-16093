package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(15  ,-62.3 ,Math.toRadians(-90)));

        while(opModeInInit()){

        }

        waitForStart();
        firstMoveToRedChamberPlace(0);
        highChamberPlace();

        //        prepareForPushTwoBlueSample();
        pushTwoRedSamples();

        prepareForClawRedSampleUp(0);
        clawRedSampleUp(0);

        moveToRedChamberPlace(3);
        highChamberPlace();

        prepareForClawRedSampleUp(10);
        clawRedSampleUp(10);

        moveToRedChamberPlace(8);
        highChamberPlace();

        //simpleParkAtObservation();
        parkAtRedObservationFromChamber();
        prepareForTeleOp();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}
