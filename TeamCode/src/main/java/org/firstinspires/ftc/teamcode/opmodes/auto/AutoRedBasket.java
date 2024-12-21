package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(-30, -62.3, Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        firstPutRedBasket();
        resetAfterRedBasketAndMoveToIntake(0,0);

        getYellowSamples();

        putRedBasket();
        resetAfterRedBasketAndMoveToIntake(10,0);

        getYellowSamples();

        putRedBasket();
        reset();

        moveAndIntakeLastBasketSampleRed();
        reset();

        putRedBasket();

        hangFromRedBasket();

//        parkAtBlueObservationFromBasket();
//        prepareForTeleOp();

        while(opModeIsActive()){
            super.update.run();
        }


//        reset();
    }


}
