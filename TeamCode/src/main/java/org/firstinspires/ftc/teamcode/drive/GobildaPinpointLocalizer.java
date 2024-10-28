package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;

import java.util.Arrays;
import java.util.List;

@Config
public class GobildaPinpointLocalizer extends TwoTrackingWheelLocalizer {
    private GoBildaPinpointDriver odo;

    public GobildaPinpointLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(new Pose2d(5,5,90),new Pose2d(-5,-5,0)));
//        this.odo = odo;
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(0,0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.update();
    }

    @Override
    public double getHeading() {
        odo.update();
        return odo.getHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        odo.update();
        return Arrays.asList(
                mmToInches(odo.getPosX()),
                mmToInches(odo.getPosY())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        odo.update();
        return Arrays.asList(
                mmToInches(odo.getVelX()),
                mmToInches(odo.getVelY())
        );
    }
    public static double mmToInches(double mm) {
        return mm/25.4;
    }
}
