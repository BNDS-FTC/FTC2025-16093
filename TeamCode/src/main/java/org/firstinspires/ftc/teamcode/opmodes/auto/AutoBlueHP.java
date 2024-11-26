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

        delay(200);

        moveToBlueChamberPlace();

        highChamberPlace();

        reset();

        pushTwoBlueSamples();

        intakeLastBlueSample();
        delay(2000);
        reset();


    }


}
