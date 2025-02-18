package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class ExperimentalAutoRedBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(-44, -60, Math.toRadians(55)));

        while(opModeInInit()){

        }

        waitForStart();
        setStartTime();

        expFirstPutRedBasket();
        expResetAfterRedBasketAndMoveToIntake(-0.3, -0.5, 0);

        expGetYellowSamples();

        putRedBasketFromGround(0,0, 0);
        expResetAfterRedBasketAndMoveToIntake(-7.2, -1.8, 0);

        expGetYellowSamples();

        putRedBasketFromGround(0,0, 0);

        moveAndIntakeLastBasketSampleRed();

        putRedBasketFromGround(0,-1, 0);


        getSamplesFromSubmersibleRed(0);

        putRedBasketFromSubmersible(0,0, -9, 0.15);


        getSamplesFromSubmersibleRedWithEmergencyAscent(3);

        putRedBasketFromSubmersible(4,-1.5, 4, 0.15);


        getSamplesFromSubmersibleRedWithEmergencyAscent(3);





//        ExpHangFromBlueBasket(); THIS DOESNT WORK!!

//        reset();

//        hangFromBlueBasket();

//        parkAtBlueObservationFromBasket();
//        prepareForTeleOp();

        while(opModeIsActive()){
            super.update.run();
        }


//        reset();
    }


}
