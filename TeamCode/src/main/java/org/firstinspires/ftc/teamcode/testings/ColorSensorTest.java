package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;

@TeleOp(name="color sensor test", group="Testing")
public class ColorSensorTest extends LinearOpMode {
    SuperStructure upper;
    NewMecanumDrive drive;
    double oldTime = 0;
    public void runOpMode(){
        upper = new SuperStructure(this,()->{},0);
        drive = new NewMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            upper.update();
            telemetry.addData("Detected Sample Color", upper.colorOfSample());
            telemetry.addData("Color Sensor Covered?", upper.colorSensorCovered());
            NormalizedRGBA rgba = upper.getColorRGBAValues();
            telemetry.addLine(String.format("rgba: %.3f %.3f %.3f %.3f",rgba.red*1000, rgba.green*1000, rgba.blue*1000, rgba.alpha*1000));
            telemetry.addData("color",upper.colorOfSample());
            telemetry.addData("distance",upper.getDistance());
            telemetry.update();


            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        }
    }
}
