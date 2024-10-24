package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;

import java.util.Locale;

/*
 * Tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    |           || |
 *    |           || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
@Disabled
public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    public static final double FORWARD_OFFSET = -3.9375; // mm; offset of the lateral wheel\

    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    private Pose2d poseVelocity = new Pose2d(0, 0, 0);

    private double last_x_pos, last_y_pos;
    private final NanoClock time;
    private double last_time, last_rotation;
    private int rev_num = 0;
    GoBildaPinpointDriver odometry;

    Telemetry telemetry;

    public Localizer(HardwareMap hardwareMap) {
        odometry = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        time = NanoClock.system();
        last_x_pos = odometry.getPosX();
        last_y_pos = odometry.getPosY();
        last_time = time.seconds();
        last_rotation = odometry.getHeading();
    }

    public static double encoderMMToInches(double value) {
        return value/25.4;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @NonNull
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    private double heading_rad_correct = 0;

    @Override
    public void update() {
        Pose2D current_pos = odometry.getPosition();
        double current_x = current_pos.getX(DistanceUnit.INCH);
        double current_y = current_pos.getY(DistanceUnit.INCH);
        double rotation = current_pos.getHeading(AngleUnit.RADIANS);

        double current_time = time.seconds();
        double corrected_rotation = rotation + Math.PI * 2 * rev_num;
        if (corrected_rotation - last_rotation > Math.PI) {
            rev_num--;
        } else if (corrected_rotation - last_rotation < -Math.PI) {
            rev_num++;
        }
        corrected_rotation = rotation + Math.PI * 2 * rev_num;

        double d_horizontal = current_x - last_x_pos;
        double d_vertical = current_y - last_y_pos;
        double d_time = last_time - current_time;
        double d_rotation = corrected_rotation - last_rotation;

        last_x_pos = current_x;
        last_y_pos = current_y;
        last_time = current_time;
        last_rotation = corrected_rotation;

        double d_x = d_horizontal;
        double d_y = d_vertical;
        Vector2d d_pos = (new Vector2d(d_x, d_y)).rotated(corrected_rotation);

        poseEstimate = new Pose2d(poseEstimate.vec().plus(d_pos), rotation);
        poseVelocity = new Pose2d(d_pos.div(d_time), odometry.getHeadingVelocity());
        odometry.update();
        telemetry.addData("Current Pos",String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", current_pos.getX(DistanceUnit.INCH), current_pos.getY(DistanceUnit.MM), current_pos.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("x_error", current_pos.getX(DistanceUnit.INCH) - poseEstimate.getX());
        telemetry.addData("y_error",current_pos.getY(DistanceUnit.INCH) - poseEstimate.getY());
        telemetry.addData("Heading_error",Math.toDegrees(current_pos.getHeading(AngleUnit.RADIANS)) - poseEstimate.getHeading());
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d poseEstimate) {
        heading_rad_correct = odometry.getHeading() - poseEstimate.getHeading();
        this.poseEstimate = poseEstimate;
        last_rotation = poseEstimate.getHeading();
    }
}