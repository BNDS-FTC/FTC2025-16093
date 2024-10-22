package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;

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
    public static final double TICKS_PER_REV = 2000;
    public static final double WHEEL_RADIUS = 0.945; // mm

    public static final double FORWARD_OFFSET = -3.9375; // mm; offset of the lateral wheel\

    public static double LATERAL_DISTANCE = 6.7;

    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    private Pose2d poseVelocity = new Pose2d(0, 0, 0);

    private double last_x_pos, last_y_pos;
    private final NanoClock time;
    private double last_time, last_rotation;
    private int rev_num = 0;
    GoBildaPinpointDriver odometry;

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
        double current_x = odometry.getPosX();
        double current_y = odometry.getPosY();
        double rotation = odometry.getHeading();
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

        double d_x = encoderMMToInches(d_horizontal);
        double d_y = encoderMMToInches(d_vertical) - d_rotation * FORWARD_OFFSET;
        Vector2d d_pos = (new Vector2d(d_x, d_y)).rotated(corrected_rotation);

        poseEstimate = new Pose2d(poseEstimate.vec().plus(d_pos), rotation);
        poseVelocity = new Pose2d(d_pos.div(d_time), odometry.getHeadingVelocity());

        odometry.update();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d poseEstimate) {
        heading_rad_correct = odometry.getHeading() - poseEstimate.getHeading();
        this.poseEstimate = poseEstimate;
        last_rotation = poseEstimate.getHeading();
    }
}