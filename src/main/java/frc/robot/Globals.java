package frc.robot;

import static frc.robot.Constants.*;

public final class Globals {
    public static String CURRENT_DRIVE_MODE = SWERVE_DRIVE;
    public static String[] CURRENT_DIRECTIONS = new String[] { FORWARDS, FORWARDS, FORWARDS, FORWARDS };
    public static boolean IS_ROTATING = false;
    public static boolean IS_RECORDING = false;
    public static boolean IS_BALANCING = false;
    public static boolean USE_OLD_SWERVE_DRIVE = false;
}
