package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp(name="color sensor test")
public class colorSensorTest extends LinearOpMode {
    public void runOpMode(){
        SuperStructure upper;
        upper = new SuperStructure(
                this,
                () -> {
                }, 0);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("RGBA",upper.getColorRGBAValues());
            telemetry.addData("res",upper.colorOfTheBlock());
            telemetry.update();
        }
    }
}
