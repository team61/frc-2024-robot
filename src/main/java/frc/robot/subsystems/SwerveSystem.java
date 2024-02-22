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
    //private boolean suppressInputs;

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
        //newTranslationVector = Vector2D.rotateByDegrees(newTranslationVector, yaw);

        double newTranslationMagnitude = newTranslationVector.magnitude();
        if (newTranslationMagnitude > 1) {
            newTranslationVector = Vector2D.scalarDivide(newTranslationVector, newTranslationMagnitude);
        }

        targetTranslationVector = newTranslationVector;

        //translationVector = Vector2D.lerp(translationVector, newTranslationVector, Constants.translationLerpFactor);
    }

    // public void updateTargetAngle(Vector2D joystickVector) {
    //     double newTargetAngle = -joystickVector.theta() + 90;
    //     if (newTargetAngle > 180) {
    //         newTargetAngle -= 360;
    //     }
    //     if (joystickVector.magnitude() >= Constants.targetAngleMinJoystickMagnitude) {
    //         // while (gyro.getYaw() - newTargetAngle > 180) {
    //         //     newTargetAngle += 360;
    //         // }
    //         // while (gyro.getYaw() - newTargetAngle < -180) {
    //         //     newTargetAngle -= 360;
    //         // }
    //         targetAngle = newTargetAngle;
    //     }
    // }

    // public void updateRotationPowerTargetted(double yaw) {
    //     double angleOffset = Math.abs(targetAngle - yaw);
    //     double newRotationPower = (angleOffset - Constants.rotationZeroThreshold) / (Constants.rotationMaxThreshold - Constants.rotationZeroThreshold);
    //     newRotationPower = Math.min(Math.max(newRotationPower, 0), 1);
    //     if (targetAngle - yaw < 0) {
    //         newRotationPower *= -1;
    //     }
    //     if (angleOffset > 180) {
    //         newRotationPower *= -1;
    //     }

    //     targetRotationPower = newRotationPower;

    //     //rotationPower = Utils.lerp(rotationPower, newRotationPower, Constants.rotationLerpFactor);
    // }

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

        //this.rotationPower = Utils.lerp(this.rotationPower, rotationPower, Constants.rotationLerpFactor);
    }

    public void forceRotationPower(double rotationPower) {
        targetRotationPower = rotationPower;
    }

    public void forceTargetAngle(Double angle) {
        targetAngle = angle;
    }

    // public void apply(DriveSystem driveSystem) {
    //     apply(driveSystem, false);
    // }

    // public void apply(DriveSystem driveSystem, boolean override) {
    //     if (!suppressInputs || override) {
    //         driveSystem.setMovement(Vector2D.scalarMultiply(translationVector, translationThrottle), rotationPower * rotationThrottle);
    //         System.out.println(Vector2D.scalarMultiply(translationVector, translationThrottle).toString());
    //     }
    // }

    // no need for supression MUHAHAHAA
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

    // public void suppress() {
    //     suppressInputs = true;
    // }

    // public void stopSuppressing() {
    //     suppressInputs = false;
    // }
}
