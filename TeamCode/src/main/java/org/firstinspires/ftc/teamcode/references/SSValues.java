package org.firstinspires.ftc.teamcode.references;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SSValues {
    public static int AUTO_ARM_OFFSET = 487;

    //****DO NOT SET ARM VALUES LARGER THAN ARM_UP****
    public static int ARM_DOWN = 0;
    public static int ARM_UP = 1086;
    public static int ARM_HANG1 = 570;
    public static int ARM_SLIGHTLY_HIGHER = 100;
    public static int ARM_ASCENT_AIM = 950;
    public static int ARM_LOWER_FROM_BASKET = 840;
    public static int ARM_ASCENT_SWING = 650;
    public static int ARM_ASCENT_END = 400;
    public static int ARM_GET_WALL_SPECIMEN = 0;
    public static int ARM_GET_WALL_SPECIMEN_UP = ARM_GET_WALL_SPECIMEN+100;


    public static int slideMaxValue = 1930;//Adjust this variable to assume that everything else changes in proportion.
    public static int SLIDE_MIN = 0;
    public static int SLIDE_HOLD_ASCENT = -100; // BE REALLY CAREFUL WITH THIS ONE OKAY???
    public static int SLIDE_OPENLOOP_LIMIT = 50;

    public static int SLIDE_SLIGHTLY_LONGER = (int)(slideMaxValue *0.1903);
    public static int SLIDE_LONGER = (int)(slideMaxValue *0.3);
    public static int SLIDE_LOW_BASKET = (int)(slideMaxValue *0.4);

    public static int SLIDE_INTAKE_NEAR = (int)(slideMaxValue *0.0955);
    public static int SLIDE_INTAKE_WALL_SPECIMEN = (int)(slideMaxValue *0.25);
    public static int SLIDE_AUTO_INTAKE_LAST_BLUE = (int)(slideMaxValue *0.54);
    public static int SLIDE_AUTO_INTAKE_LAST_RED = (int)(slideMaxValue *0.50);
    public static int SLIDE_AUTO_INTAKE_FIRST= (int)(slideMaxValue *0.5538);
    public static int SLIDE_AUTO_INTAKE_YELLOW = (int)(slideMaxValue *0.4);
    public static int SLIDE_SWITCH_LIMIT = (int)(slideMaxValue *0.3);
    public static int SLIDE_INTAKE_FAR = (int)(slideMaxValue *0.38);
    public static int SLIDE_HIGH_CHAMBER_AIM_AUTO = (int)(slideMaxValue *0.305);
    public static int SLIDE_HIGH_CHAMBER_AIM_TELEOP = (int)(slideMaxValue *0.30);
    public static int SLIDE_HIGH_CHAMBER_PLACE = (int)(slideMaxValue *0.57);
    public static int SLIDE_HIGH_CHAMBER_PLACE_AUTO = (int)(slideMaxValue *0.56);
    public static int SLIDE_ASCENT_UP = (int)(slideMaxValue *0.8578);
    public static int SLIDE_ASCENT_DOWN = (int)(slideMaxValue *0.3);
    public static int SLIDE_MAX = slideMaxValue;

    public static double WRIST_DEFAULT = 1;

    public static double WRIST_INTERMEDIATE = 0.6;
    public static double WRIST_HIGH_CHAMBER = 0.95;
    public static double WRIST_RELEASE_AUTO = 0.72;
    public static double WRIST_RELEASE_TELEOP = 0.8;
    public static double WRIST_RELEASE_EXTRA = 0.75;
    public static double WRIST_ABOVE_SAMPLES = 0.49;
    public static double WRIST_INTAKE = 0.36;
    public static double WRIST_INTAKE_SPECIMEN_DONTUSETHIS = 0.38;
    public static double WRIST_HIGH_CHAMBER_RESET = 0.41;
    public static double WRIST_INTAKE_WALL_SPECIMEN = 0.71;

    public static double TAIL_DEFAULT = 0.83;
    public static double TAIL_CHAMBER = 0.27;
    public static double TAIL_AUTO_POS = 0.83;

    public static double GRAB_DEFAULT = 0.66;
    public static double GRAB_OPEN = 0.8;
    public static double GRAB_CLOSED = 0.38;
    public static double GRAB_CLOSED_WITHOUT_CAP = 0.4;
    public static double AUTO_GRAB_CLOSED = 0.4;

    public static double ASCENT_RIGHT_DOWN_A_LITTLE = 0.75;
    public static double ASCENT_RIGHT_DOWN_SOME_MORE = 0.7;
    public static double ASCENT_RIGHT_DOWN = 0.5;
    public static double ASCENT_RIGHT_UP = 0.88;

    public static double ASCENT_LEFT_DOWN_A_LITTLE = 0.25;
    public static double ASCENT_LEFT_DOWN_SOME_MORE = 0.3;
    public static double ASCENT_LEFT_DOWN = 0.52;
    public static double ASCENT_LEFT_UP = 0.12;


    public static double CONTINUOUS_SPIN = 1;
    public static double CONTINUOUS_STOP = 0.5;
//    public static final double CONTINUOUS_STOP_OPPOSITE=0.77;
    public static double CONTINUOUS_SPIN_OPPOSITE = 0;

//    public static final double SLIDE_LOCK_DEFAULT = 0;
//    public static final double SLIDE_LOCK_LOCKED_TIGHT = 0.14;
//    public static final double SLIDE_LOCK_LOCKED_NORMAL = 0.13;
}