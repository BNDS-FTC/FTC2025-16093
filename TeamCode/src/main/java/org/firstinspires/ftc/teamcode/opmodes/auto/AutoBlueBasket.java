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

        initHardware(new Pose2d(30, 62.3, Math.toRadians(-90)));

        while(opModeInInit()){

        }

        waitForStart();
        firstPutBlueBasket();
        resetAfterBlueBasketAndMoveToIntake(-0.5,0);

        getYellowSamples();

        putBlueBasket();
        resetAfterBlueBasketAndMoveToIntake(10,-2);

        getYellowSamples();

        putBlueBasket();
        reset();

        moveAndIntakeLastBasketSampleBlue();
        reset();

        putBlueBasket();

        hangFromBlueBasket();

//        parkAtBlueObservationFromBasket();
//        prepareForTeleOp();

        while(opModeIsActive()){
            super.update.run();
        }


//        reset();
    }


}
