package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoMaster;

@TeleOp (group = "Testing")
@Config
public class TestAutoPIDWithSuperstructure extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(new Pose2d(testPIDx,testPIDy,Math.toRadians(testPIDheading)));

        waitForStart();

        autoArmTest();
        while(opModeIsActive()){
            testAutoPID();
            super.update.run();
        }

    }
}
