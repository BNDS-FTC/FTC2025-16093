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

        // TODO: THIS IS BROKEN!
        initHardware(new Pose2d(-15  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();

        moveToBlueChamberAim();
        highChamberAim();
        moveToBlueChamberPlace(0);
        highChamberPlace();

        reset();

        pushTwoBlueSamples();
        intakeLastBlueSample();
        reset();
        placeLastBlueSampleAtHP();

        clawBlueSampleUp();
        moveToBlueChamberAim();
        highChamberAim();
        moveToBlueChamberPlace(4);
        highChamberPlace();
        prepareToClawIntakeBlueSpecimen(-2);

        clawBlueSampleUp();
        moveToBlueChamberAim();
        highChamberAim();
        moveToBlueChamberPlace(8);
        highChamberPlace();
        prepareToClawIntakeBlueSpecimen(-3);
    }


}
