package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoMaster;

@TeleOp (group = "Testing")
@Disabled
public class TestAutoPIDWithSuperstructure extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(new Pose2d(testPIDx,testPIDy,Math.toRadians(testPIDheading)));

        waitForStart();

        autoArmTest();
        while(opModeIsActive()){
            testAutoPID();
            super.update.run();
        }

    }
}
