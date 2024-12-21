package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        firstMoveToBlueChamberPlace(1);
        highChamberPlace();

        //        prepareForPushTwoBlueSample();
        pushTwoBlueSamples();

        prepareForClawBlueSampleUp(0,0);
        clawBlueSampleUp(0,0);

        moveToBlueChamberPlace(4);
        highChamberPlace();

        prepareForClawBlueSampleUp(17,-0.5);
        clawBlueSampleUp(17,-0.5);

        moveToBlueChamberPlace(8);
        highChamberPlace();

        prepareForClawBlueSampleUp(5,0.3);
        clawBlueSampleUp(5,0.3);
        prepareForTeleOpBlue();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}
