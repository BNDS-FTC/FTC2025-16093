package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name =  "Generic Auto Test", group = "Testing")
public class AutoGenericTest extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        // TODO: THIS IS ALL WRONG???
        initHardware();

        while(opModeInInit()){

        }

        moveToHighChamber();

        highChamberPlace();

        reset();

        moveToPushSample1();

        pushSample();

    }
}