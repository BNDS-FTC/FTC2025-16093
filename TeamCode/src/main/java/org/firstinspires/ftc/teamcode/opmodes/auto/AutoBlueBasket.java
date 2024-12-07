package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoBlueBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        // TODO: THIS IS BROKEN!
        initHardware(new Pose2d(15, 62.3, Math.toRadians(-90)));

        while(opModeInInit()){

        }

        waitForStart();
        firstPutBlueBasket();
        resetAfterBlueBasket();

        getYellowSamples(0);
        reset();

        putBlueBasket();
        resetAfterBlueBasket();

        getYellowSamples(10);
        reset();

        putBlueBasket();
        resetAfterBlueBasket();

        moveToGetLastYellowSample();
        intakeLastSample();
        reset();

        putBlueBasket();
        reset();

        prepareForTeleOp();


//        reset();
    }


}
