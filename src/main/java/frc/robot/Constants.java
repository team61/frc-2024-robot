package frc.robot;

public final class Constants {
    public static final int joystickPort1 = 1;
    public static final int joystickPort2 = 3;
    public static final int joystickPort3 = 2;
    public static final int joystickPort4 = 0;

    public static final double MAX_ROTATION_VOLTAGE = 1.5;
    public static final double MAX_DRIVE_VOLTAGE = 12;
    public static final double SLOWDOWN_COEFFICIENT = 0.5;

    public static final double ENCODER_UNITS_PER_ROTATION = 360;
    public static final double OVER_ROTATION_PADDING = 0.25;
    public static final double PRECISION_ROTATION_PADDING = 8;

    public static final String FORWARDS = "forwards";
    public static final String SIDEWAYS = "sideways";
    public static final String MIDDLE = "middle";

    public static final String SWERVE_DRIVE = "swervedrive";
    public static final String TANK_DRIVE = "tankdrive";

    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }
}
