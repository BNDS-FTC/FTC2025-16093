package org.firstinspires.ftc.teamcode.opmodes.auto;

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
        moveToBlueChamberPlace(6);
        highChamberPlace();
        prepareToClawIntakeBlueSpecimen(-2);

        clawBlueSampleUp();
        moveToBlueChamberAim();
        highChamberAim();
        moveToBlueChamberPlace(12);
        highChamberPlace();
        prepareToClawIntakeBlueSpecimen(-2);
    }


}
