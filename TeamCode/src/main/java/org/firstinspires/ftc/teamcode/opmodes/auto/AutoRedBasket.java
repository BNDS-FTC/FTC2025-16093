package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedBasket extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = NEGATIVE;

        // TODO: THIS IS ALL WRONG???
        initHardware();

        while(opModeInInit()){

        }

        moveToHighChamber();

        reset();

        intakeFloorSample();

    }
}