package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class ExperimentalAutoBlueBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

//        initHardware(new Pose2d(30, 62.3, Math.toRadians(-90)));
        initHardware(new Pose2d(44, 60, Math.toRadians(-135)));

        while(opModeInInit()){

        }

        waitForStart();
        setStartTime();

        expFirstPutBlueBasket();
        expResetAfterBlueBasketAndMoveToIntake(0.5, 0.5, -2);

        expGetYellowSamples();

        putBlueBasketFromGround(0,0, 0);
        expResetAfterBlueBasketAndMoveToIntake(9, 1.2, -2);

        expGetYellowSamples();

        putBlueBasketFromGround(0,0, 0);

        moveAndIntakeLastBasketSampleBlue();

        putBlueBasketFromGround(0,1, 0);

        getSamplesFromSubmersibleBlue(0);

        putBlueBasketFromSubmersible(-1,2, 0.3);

        getSamplesFromSubmersibleBlueWithEmergencyAscent(3);


        putBlueBasketFromSubmersible(-1,2, 0.35);

        getSamplesFromSubmersibleBlueWithEmergencyAscent(-3);





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
