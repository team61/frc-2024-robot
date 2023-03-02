package frc.robot;

public final class Constants {
    public static final int joystickPort1 = 1;
    public static final int joystickPort2 = 3;
    public static final int joystickPort3 = 2;
    public static final int joystickPort4 = 0;

    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;

    public static final double MAX_ROTATION_VOLTAGE = 1.5;
    public static final double MAX_DRIVE_VOLTAGE = 12;
    public static final double MIN_ELEVATOR_VOLTAGE = 4;
    public static final double MAX_ELEVATOR_VOLTAGE = 12;
    public static final double ELEVATOR_NEGATIVE_LIMIT = -3;
    public static final double MIN_ARM_VOLTAGE = 0.5;
    public static final double MAX_ARM_VOLTAGE = 4;
    public static final double SLOWDOWN_COEFFICIENT = 0.5;

    public static final double ENCODER_UNITS_PER_ROTATION = 360;
    public static final double OVER_ROTATION_PADDING = 0.25;
    public static final double PRECISION_ROTATION_PADDING = 1;

    public static final double ELEVATOR_MIN_POSITION = 0;
    public static final double ELEVATOR_MAX_UNEXTENDED_POSITION = 270000;
    public static final double ELEVATOR_MAX_POSITION = 740000;
    public static final double ARM_MIN_POSITION = -150000;
    public static final double ARM_MAX_POSITION = 0;
    public static final double ARM_EXTENSION_THRESHOLD = -36000;
    public static final double ARM_MIN_LOWER_EXTENSION = -36000;

    public static final double ELEVATOR_HUMAN_PIECE_POSITION = 183000;
    public static final double ARM_HUMAN_PIECE_POSITION = -2600;
    public static final double ELEVATOR_FLOOR_PIECE_POSITION = 685000;
    public static final double ARM_FLOOR_PIECE_POSITION = -43900;

    public static final String FORWARDS = "forwards";
    public static final String BACKWARDS = "backwards";
    public static final String SIDEWAYS = "sideways";
    public static final String DIAGONAL = "diagonal";
    public static final String MIDDLE = "middle";
    public static final String OUTER = "outer";

    public static final String HUMAN = "human";
    public static final String FLOOR = "floor";

    public static final String SWERVE_DRIVE = "swervedrive";
    public static final String TANK_DRIVE = "tankdrive";

    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }
}
