package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.ArmAction;
import org.firstinspires.ftc.teamcode.actions.GrabAction;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.actions.WristAction;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

@TeleOp(name="color sensor test", group="Testing")
public class ColorSensorTest extends LinearOpMode {
    private final double POWER=0.5;
    private ColorSensor color;
    private DistanceSensor distance;
    private final List<Integer> cachedColor = new ArrayList<>(Arrays.asList(0,0,0,-1));

    public void runOpMode(){
        color = hardwareMap.get(ColorSensor.class, "color");
        distance = hardwareMap.get(DistanceSensor.class, "color");
        waitForStart();

        while(opModeIsActive()){
            currentAlpha = color.alpha();
            color.enableLed(false);

            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));

            telemetry.addData("RGBA",getColorRGBAValues(5));
            telemetry.addData("Color",colorOfSample());
            telemetry.addData("Alpha Adjusted Color",colorOfSample());
            telemetry.addData("Color sensor covered?",colorSensorCovered());
            telemetry.update();
        }
    }

    public List<Integer> getColorRGBAValues(int threshold) {
        if (cachedColor.get(3)==-1){
            cachedColor.clear();
            cachedColor.add(0,color.red());
            cachedColor.add(1,color.green());
            cachedColor.add(2,color.blue());
            cachedColor.add(3,color.alpha());
            return cachedColor;
        }else{
            int a=color.alpha();
            if (Math.abs(cachedColor.get(3) - a) > threshold) {
                cachedColor.clear();
                cachedColor.add(0, color.red());
                cachedColor.add(1, color.green());
                cachedColor.add(2, color.blue());
                cachedColor.add(3, a);
            }
            return cachedColor;
        }
    }

    public boolean colorSensorCovered(){
        return color.alpha() > 35 && distance.getDistance(DistanceUnit.CM) < 4.3;//90
//        List<Integer> rgbaValues = getColorRGBAValues();
//        return Collections.max(rgbaValues)>90;
    }

    public boolean colorSensorCoveredAuto(){
        return color.alpha() > 60;//90
//        List<Integer> rgbaValues = getColorRGBAValues();
//        return Collections.max(rgbaValues)>90;
    }

    private int redThreshold = 35;
    private int yellowThreshold = 35;
    private int blueThreshold = 30;
    private int indexOfMaxRGB = 0;
    private int currentAlpha = 0;
    private int currentRed = 0;
    private int currentGreen = 0;
    private int currentBlue = 0;
    List<Integer> rgbaValues;

    public String alphaAdjustedSampleColor(){
        rgbaValues = getColorRGBAValues(5);//color should not change...?
        if(colorSensorCovered()) {
            indexOfMaxRGB = rgbaValues.indexOf(Collections.max(rgbaValues));
            currentRed = rgbaValues.get(0);
            currentGreen = rgbaValues.get(1);
            currentBlue = rgbaValues.get(2);
            if (indexOfMaxRGB == 0 && compareColorDiff(currentRed, currentGreen, currentBlue) && currentAlpha > redThreshold) {
                return "red";
            } else if (indexOfMaxRGB == 1 && compareColorDiff(currentGreen, currentRed, currentBlue) && currentAlpha > yellowThreshold) {
                return "yellow";
            } else if (indexOfMaxRGB == 2) {
                return "blue";
            }else if(indexOfMaxRGB == 1 && compareColorDiff(currentGreen, currentBlue, currentRed)){
                return "";
            }
        }
        return "";
    }


    public String colorOfSample(){
        if(colorSensorCovered()){
            rgbaValues = getColorRGBAValues(5);
            if(colorSensorCovered()){
                indexOfMaxRGB = rgbaValues.indexOf(Collections.max(rgbaValues));
                currentRed = rgbaValues.get(0);
                currentGreen = rgbaValues.get(1);
                currentBlue = rgbaValues.get(2);
                if(indexOfMaxRGB == 0 && compareColorDiff(currentRed,currentGreen,currentBlue)){
                    return "red";
                }else if(indexOfMaxRGB == 1 && compareColorDiff(currentGreen,currentRed,currentBlue)){
                    return"yellow";
                }else if(indexOfMaxRGB == 2){
                    return "blue";
                }else if(indexOfMaxRGB == 1 && compareColorDiff(currentGreen, currentBlue, currentRed)){
                    return "";
                }
            }
            return "unknown";
        }
        return "No sample detected";
    }

    private boolean compareColorDiff(int target, int closeTo, int farFrom){
        if(Math.abs(target-closeTo)< Math.abs(target-farFrom)){
            return true;
        }
        return false;
    }

}
