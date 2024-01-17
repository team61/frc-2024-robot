package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Vector2D;

public class DriveSystem {
    private static DriveSystem system;

    public DriveModule[] modules;

    private DriveSystem() {
        modules = new DriveModule[4];

        for (int i = 0; i < 4; i++) {
            modules[i] = new DriveModule(Constants.driveMotorNumbers[i], Constants.angleMotorNumbers[i], Constants.angleEncoderNumbers[i], Constants.driveModulePositions[i]);
        }
    }

    public static DriveSystem get() {
        if (system == null) {
            system = new DriveSystem();
        }

        return system;
    }

    public void enable() {
        for (DriveModule module : modules) {
            module.enable();
        }
    }

    public void disable() {
        for (DriveModule module : modules) {
            module.disable();
        }
    }

    public void callibrateAngles() {
        for (DriveModule module : modules) {
            module.calibrateAngle();
        }
    }

    public void setMovement(Vector2D translationVector, double rotationPower) {
        for (DriveModule module : modules) {
            module.setMovement(translationVector, rotationPower);
        }
    }
}
