package frc.robot;

public final class Constants {
    public static final int joystickPort1 = 0;
    public static final int joystickPort2 = 1;
    public static final int joystickPort3 = 2;
    public static final int joystickPort4 = 3;

    public static final double MAX_ROTATION_VOLTAGE = 1.5;
    public static final double MAX_DRIVE_VOLTAGE = 12;
    public static final double SLOWDOWN_COEFFICIENT = 0.5;

    public static final double SWERVE_UNITS_PER_ROTATION = 26100;
    public static final double SWERVE_UNITS_PADDING = 100;

    public static final String FORWARDS = "forwards";
    public static final String SIDEWAYS = "sideways";
    public static final String MIDDLE = "middle";

    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }
}
