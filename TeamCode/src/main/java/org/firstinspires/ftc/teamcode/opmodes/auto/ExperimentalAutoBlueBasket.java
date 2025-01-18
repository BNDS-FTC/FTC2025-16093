package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class ExperimentalAutoBlueBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(new Pose2d(44, 58, Math.toRadians(-135)));

        while(opModeInInit()){

        }

        waitForStart();
        expFirstPutBlueBasket();
        expResetAfterBlueBasketAndMoveToIntake(-0.5,0);

        expGetYellowSamples();

        putBlueBasket();
        expResetAfterBlueBasketAndMoveToIntake(10,-2);

        expGetYellowSamples();

        putBlueBasket();
        reset();

        moveAndIntakeLastBasketSampleBlue();
        reset();

        putBlueBasket();
        reset();

        getSamplesFromSubmersibleBlue();

        delay(200);

//        hangFromBlueBasket();

//        parkAtBlueObservationFromBasket();
//        prepareForTeleOp();

        while(opModeIsActive()){
            super.update.run();
        }


//        reset();
    }


}
