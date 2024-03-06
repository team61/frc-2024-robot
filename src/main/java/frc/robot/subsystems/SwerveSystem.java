package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Vector2D;

public class SwerveSystem {
    private static SwerveSystem system;

    public Vector2D translationVector = Vector2D.zero, targetTranslationVector = Vector2D.zero;
    private Double targetAngle;
    private double rotationPower, targetRotationPower;
    public double translationThrottle;
    public double rotationThrottle;

    private SwerveSystem() {
        translationThrottle = Constants.translationThrottle;
        rotationThrottle = Constants.rotationThrottle;
    }

    public static SwerveSystem get() {
        if (system == null) {
            system = new SwerveSystem();
        }

        return system;
    }

    public void updateTranslationVector(Vector2D joystickVector, double yaw) {
        Vector2D newTranslationVector = joystickVector;

        double newTranslationMagnitude = newTranslationVector.magnitude();
        if (newTranslationMagnitude > 1) {
            newTranslationVector = Vector2D.scalarDivide(newTranslationVector, newTranslationMagnitude);
        }

        targetTranslationVector = newTranslationVector;
    }

    public void updateRotationPower(double rotationPower) {
        if (targetAngle != null) {
            targetAngle += rotationPower * rotationThrottle * Constants.rotationTargetShiftFactor;

            if (targetAngle < -180) {
                targetAngle += 360;
            }
            else if (targetAngle > 180) {
                targetAngle -= 360;
            }
        }
    }

    public void forceRotationPower(double rotationPower) {
        targetRotationPower = rotationPower;
    }

    public void forceTargetAngle(Double angle) {
        targetAngle = angle;
    }

    public void update(double yaw) {
        if (targetAngle != null) {
            double angleOffset = Math.abs(targetAngle - yaw);
            double adjustedAngleOffset = angleOffset;
            if (adjustedAngleOffset > 180) {
                adjustedAngleOffset = 360 - adjustedAngleOffset;
            }
            double newRotationPower = (adjustedAngleOffset - Constants.rotationZeroThreshold) / (Constants.rotationMaxThreshold - Constants.rotationZeroThreshold);
            newRotationPower = Math.min(Math.max(newRotationPower, 0), 1);
            if (targetAngle - yaw < 0) {
                newRotationPower *= -1;
            }
            if (angleOffset > 180) {
                newRotationPower *= -1;
            }
    
            targetRotationPower = newRotationPower;
        }
        else {
            targetRotationPower = 0;
        }
        
        translationVector = Vector2D.lerp(translationVector, targetTranslationVector, Constants.translationLerpFactor);
        rotationPower = Utils.lerp(rotationPower, targetRotationPower, Constants.rotationLerpFactor);
        
        DriveSystem.get().setMovement(Vector2D.scalarMultiply(Vector2D.rotateByDegrees(translationVector, yaw), translationThrottle), rotationPower * rotationThrottle);
    }
}
