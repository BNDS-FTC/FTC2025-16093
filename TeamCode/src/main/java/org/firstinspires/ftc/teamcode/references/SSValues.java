package org.firstinspires.ftc.teamcode.references;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SSValues {
    public static final int AUTO_ARM_OFFSET = 200;

    //****DO NOT SET ARM VALUES LARGER THAN 1100****
    public static final int ARM_DOWN = 0;
    public static final int ARM_UP = 1050;
    public static final int ARM_HANG1 = 680;


    public static int maxValue = 1690;//Adjust this variable to assume that everything else changes in proportion.
    public static final int SLIDE_MIN = 0;
    public static final int SLIDE_SLIGHTLY_LONGER = (int)(maxValue*0.1903);
    public static final int SLIDE_INTAKE_NEAR = (int)(maxValue*0.0955);
    public static final int SLIDE_AUTO_INTAKE_LAST= (int)(maxValue*0.5026);
    public static final int SLIDE_AUTO_INTAKE_YELLOW = (int)(maxValue*0.3246);
    public static final int SLIDE_INTAKE_FAR = (int)(maxValue*0.6494);
    public static final int SLIDE_HIGH_CHAMBER_AIM = (int)(maxValue*0.6292);
    public static final int SLIDE_HIGH_CHAMBER_PLACE = (int)(maxValue*0.4487);
    public static final int SLIDE_ASCENT_UP = (int)(maxValue*0.8578);
    public static final int SLIDE_ASCENT_DOWN = 50;
    public static final int SLIDE_MAX = maxValue;

    public static final double WRIST_DEFAULT = 0.99;
    public static final double WRIST_RELEASE = 0.75;
    public static final double WRIST_ABOVE_SAMPLES = 0.25;

    public static final double WRIST_INTAKE = 0.1;
    public static final double WRIST_HIGH_CHAMBER = 0.3;

    public static final double GRAB_DEFAULT = 0.64;
    public static final double GRAB_OPEN = 0.8;
    public static final double GRAB_CLOSED = 0.49;

    public static final double CLAW_LEFT_OPEN = 0;
    public static final double CLAW_LEFT_CLOSE = 0.35;
    public static final double CLAW_RIGHT_OPEN = 0.9;
    public static final double CLAW_RIGHT_CLOSE = 0.55;


    public static final double CONTINUOUS_SPIN = 0;
    public static final double CONTINUOUS_STOP = 0.22;
    public static final double CONTINUOUS_STOP_OPPOSITE=0.77;
    public static final double CONTINUOUS_SPIN_OPPOSITE = 1;
}