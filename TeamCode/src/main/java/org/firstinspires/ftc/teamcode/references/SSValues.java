package org.firstinspires.ftc.teamcode.references;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SSValues {
    //****DO NOT SET ARM VALUES LARGER THAN 1750****

    public static final int ARM_DEFAULT = 0;
    public static final int ARM_SOFTENED = 200;
    public static final int ARM_INTAKE_NEAR = 150;
    public static final int ARM_INTAKE_FAR = 300;
    public static final int ARM_RELEASE_CHAMBER_LOW = 800, ARM_RELEASE_CHAMBER_HIGH = 1000;
    public static final int ARM_UP = 1400;
    public static final int ARM_RELEASE_BOX_HIGH = 1700, ARM_RELEASE_BOX_LOW = 1700;
    public static final int ARM_MAX = 1750;


    public static final int SLIDE_MIN = 0;
    public static final int SLIDE_MIDDLE = 296;
    public static final int SLIDE_MAX = 1400;

    public static final double WRIST_DEFAULT = 0.99;
    public static final double WRIST_RELEASE = 0.99;
    public static final double WRIST_INTAKE_NEAR = 0.1;
    public static final double WRIST_INTAKE_FAR = 0.1;

    public static final double GRAB_DEFAULT = 0.4;
    public static final double GRAB_OPEN = 0.6;
    public static final double GRAB_CLOSED = 0.32;

    public static final double CONTINUOUS_SPIN = 0;
    public static final double CONTINUOUS_STOP = 0.3;
    public static final double CONTINUOUS_STOP_OPPOSITE=0.5;
    public static final double CONTINUOUS_SPIN_OPPOSITE = 0.8;
}
