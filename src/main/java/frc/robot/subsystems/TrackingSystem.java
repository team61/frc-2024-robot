package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
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
    public AHRS ahrs;
    public WPI_Pigeon2 pigeon;

    Vector2D position;
    double yaw;

    private TrackingSystem() {
        // limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        // limelightBotPose = limelightTable.getEntry("botpose");
        ahrs = new AHRS(Port.kMXP);
        pigeon = new WPI_Pigeon2(Constants.pigeonNumber);
        
        position = new Vector2D();

        //ahrs.calibrate();
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

        //System.out.println(gyro.isConnected() + ", " + gyro.getYaw());

        // if (!ahrs.isConnected()) {
        //     System.out.println("Gyro disconnected!");
        // }

        // return ahrs.getYaw();

        double yaw = pigeon.getYaw();

        while (yaw > 180) {
            yaw -= 360;
        }
        
        while (yaw < -180) {
            yaw += 360;
        }

        return -yaw;
    }

    public void calibrateGyro(double angle) {
        //gyro.reset();
        //gyro.setAngleAdjustment(angle);

        // ahrs.reset();
        // ahrs.setAngleAdjustment(ahrs.getYaw() + angle);

        pigeon.setYaw(-angle);
        System.out.print("Calibrated");
    }
}
