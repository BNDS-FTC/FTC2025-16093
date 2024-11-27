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
        initHardware();

        while(opModeInInit()){

        }

        waitForStart();

        moveToBlueChamberAim();

        highChamberAim();

        delay(100);

        moveToBlueChamberPlace();

        highChamberPlace();

        reset();

        pushTwoBlueSamples();

        intakeLastBlueSample();

        reset();

        placeLastBlueSampleAtHP();

//        for(int i = 0; i < 2; i++) {
//            clawBlueSampleUp();
//            moveToBlueChamberAim();
//            highChamberAim();
//            delay(100);
//            moveToBlueChamberPlace();
//            highChamberPlace();
//            clawIntakePlace();
//        }

        clawBlueSampleUp();
        moveToBlueChamberAim();
        highChamberAim();
        delay(100);
        moveToBlueChamberPlace();
        highChamberPlace();
        clawIntakePlace();

        clawBlueSampleUp();
        moveToBlueChamberAim();
        highChamberAim();
        delay(100);
        moveToBlueChamberPlace();
        highChamberPlace();
        clawIntakePlace();
    }


}
