package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Vector2D;

public class TrackingSystem {
    private static TrackingSystem system;

    NetworkTable limelightTable;
    NetworkTableEntry limelightBotPose;
    AHRS gyro;

    Vector2D position;
    double yaw;

    private TrackingSystem() {
        // limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        // limelightBotPose = limelightTable.getEntry("botpose");
        gyro = new AHRS(Port.kMXP);
        
        position = new Vector2D();

        gyro.calibrate();
    }

    public static TrackingSystem get() {
        if (system == null) {
            system = new TrackingSystem();
        }

        return system;
    }

    public void update() {
        Vector2D positionReading = new Vector2D();
        double yawReading = 0;
        double[] botPose = limelightBotPose.getDoubleArray(new double[6]);
        positionReading.x = -botPose[1];
        positionReading.y = botPose[0];
        yawReading = -botPose[5];

        if (positionReading.magnitude() > 0.01) {
            position = Vector2D.lerp(position, positionReading, Constants.trackingLerpFactor);
            yaw = Utils.lerp(yaw, yawReading, Constants.trackingLerpFactor);
        }
    }
    
    public Vector2D getPosition() {
        return position;
    }

    public double getYaw() {
        //return yaw;

        return gyro.getYaw();
    }

    public void resetGyro() {
        gyro.reset();
    }
}
