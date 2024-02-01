package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Vector2D;

public class SwerveSystem {
    private static SwerveSystem system;

    public Vector2D translationVector = Vector2D.zero;
    private double targetAngle;
    private double rotationPower;
    private boolean suppressInputs;

    private SwerveSystem() {
        //initialize properties
    }

    public static SwerveSystem get() {
        if (system == null) {
            system = new SwerveSystem();
        }

        return system;
    }

    public void updateTranslationVector(Vector2D joystickVector, double yaw) {
        translationVector = joystickVector;
        translationVector = Vector2D.rotateByDegrees(translationVector, yaw);

        double translationMagnitude = translationVector.magnitude();
        if (translationMagnitude > 1) {
            translationVector = Vector2D.scalarDivide(translationVector, translationMagnitude);
        }
    }

    public void updateTargetAngle(Vector2D joystickVector) {
        double newTargetAngle = -joystickVector.theta() + 90;
        if (newTargetAngle > 180) {
            newTargetAngle -= 360;
        }
        if (joystickVector.magnitude() >= Constants.targetAngleMinJoystickMagnitude) {
            // while (gyro.getYaw() - newTargetAngle > 180) {
            //     newTargetAngle += 360;
            // }
            // while (gyro.getYaw() - newTargetAngle < -180) {
            //     newTargetAngle -= 360;
            // }
            targetAngle = newTargetAngle;
        }
    }

    public void updateRotationPowerTargetted(double yaw) {
        double angleOffset = Math.abs(targetAngle - yaw);
        rotationPower = (angleOffset - Constants.rotationZeroThreshold) / (Constants.rotationMaxThreshold - Constants.rotationZeroThreshold);
        rotationPower = Math.min(Math.max(rotationPower, 0), 1);
        if (targetAngle - yaw < 0) {
            rotationPower *= -1;
        }
        if (angleOffset > 180) {
            rotationPower *= -1;
        }
    }

    public void throttleTranslation(double factor) {
        translationVector = Vector2D.scalarMultiply(translationVector, factor);
    }

    public void throttleRotation(double factor) {
        rotationPower *= factor;
    }

    public void updateRotationPowerLinear(double rotationPower) {
        this.rotationPower = rotationPower;
    }

    public void apply(DriveSystem driveSystem) {
        apply(driveSystem, false);
    }

    public void apply(DriveSystem driveSystem, boolean override) {
        if (!suppressInputs || override) {
            driveSystem.setMovement(translationVector, rotationPower);
        }
    }

    public void suppress() {
        suppressInputs = true;
    }

    public void stopSuppressing() {
        suppressInputs = false;
    }
}
