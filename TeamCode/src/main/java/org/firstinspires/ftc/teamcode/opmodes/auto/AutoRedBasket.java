package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRedBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(-44, -60, Math.toRadians(55)));

        while(opModeInInit()){

        }

        waitForStart();
        setStartTime();

        expFirstPutRedBasket();
        expResetAfterRedBasketAndMoveToIntake(0, -0.5, 10);

        expGetYellowSamples();

        putRedBasketFromGround(0,0, 0);
        expResetAfterRedBasketAndMoveToIntake(-6.2, -1.8, 10);

        expGetYellowSamples();

        putRedBasketFromGround(0,0, 0);

        moveAndIntakeLastBasketSampleRed();

        putRedBasketFromGround(0,-1, 0);


        getSamplesFromSubmersibleRed(-20);

        putRedBasketFromSubmersible(0,-1, 0, 0.15);


        getSamplesFromSubmersibleRedWithEmergencyAscent(0);

        putRedBasketFromSubmersible(0,-1, 0, 0.15);


        parkToRedSumbersible();




        while(opModeIsActive()){
            super.update.run();
        }


    }


}
