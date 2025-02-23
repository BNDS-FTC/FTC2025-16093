package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class AutoBlueBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(44, 60, Math.toRadians(-135)));

        while(opModeInInit()){

        }

        waitForStart();
        setStartTime();

        expFirstPutBlueBasket();
        expResetAfterBlueBasketAndMoveToIntake(0, 0.5, 0);

        expGetYellowSamples();

        putBlueBasketFromGround(0,0, 0);
        expResetAfterBlueBasketAndMoveToIntake(7.2, 1.8, 0);

        expGetYellowSamples();

        putBlueBasketFromGround(0,0, 0);

        moveAndIntakeLastBasketSampleBlue();

        putBlueBasketFromGround(-1,0, 0);


        getSamplesFromSubmersibleBlueWithEmergencyAscent(0, 30, 0);

        putBlueBasketFromSubmersible(0,1.5, 4);


        getSamplesFromSubmersibleBlueWithEmergencyAscent(30, -10, -20);

        putBlueBasketFromSubmersible(-2.5,1.5, -4);


        parkToBlueSumbersible();

        while(opModeIsActive()){
            super.update.run();
        }


    }


}
