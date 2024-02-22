package frc.robot.subsystems;

import lib.components.LogitechJoystick;

import frc.robot.Constants;
import frc.robot.Vector2D;

public class InputSystem {
    private static InputSystem system;

    public LogitechJoystick[] joysticks;

    //private int throttlePreset;

    private InputSystem() {
        joysticks = new LogitechJoystick[Constants.joystickNumbers.length];

        for (int i = 0; i < Constants.joystickNumbers.length; i++) {
            joysticks[i] = new LogitechJoystick(Constants.joystickNumbers[i]);
        }

        //throttlePreset = Constants.defaultThrottlePreset;
    }

    public static InputSystem get() {
        if (system == null) {
            system = new InputSystem();
        }

        return system;
    }

    //swerve system

    public Vector2D getTranslationVector() {
        return getJoystickVector(0);
    }

    public double getRotationPowerLinear() {
        return getJoystickVector(1).x;
    }

    public Vector2D getTargetAngleVector() {
        return joysticks[1].getVector();
    }

    // public void updateThrottle() {
    //     if (joysticks[0].btn_12.getAsBoolean()) {
    //         throttlePreset = 0;
    //     }
    //     else if (joysticks[0].btn_10.getAsBoolean()) {
    //         throttlePreset = 1;
    //     }
    //     else if (joysticks[0].btn_8.getAsBoolean()) {
    //         throttlePreset = 2;
    //     }
    //     else if (joysticks[0].btn_11.getAsBoolean()) {
    //         throttlePreset = 3;
    //     }
    //     else if (joysticks[0].btn_9.getAsBoolean()) {
    //         //throttlePreset = 4;
    //     }
    //     else if (joysticks[0].btn_7.getAsBoolean()) {
    //         //throttlePreset = 5;
    //     }
    // }

    // public double getTranslationThrottle() {
    //     return Constants.throttlePresets[throttlePreset].translationFactor;
    // }

    // public double getRotationThrottle() {
    //     return Constants.throttlePresets[throttlePreset].rotationFactor;
    // }

    public boolean getForcedRotationMode() {
        return joysticks[1].btn_2.getAsBoolean();
    }

    //arm system

    public double getElevatorPower() {
        return getJoystickVector(2).y;
    }

    public double getArmPower() {
        return -getJoystickVector(3).y;
    }

    public boolean getHandPickupButton() {
        return joysticks[3].getPOVDown();
    }

    public boolean getHandReleaseButton() {
        return joysticks[3].getPOVUp();
    }

    public boolean getBalancerEngageButton() {
        return joysticks[2].getPOVUp();
    }

    public boolean getBalancerDisengageButton() {
        return joysticks[2].getPOVDown();
    }

    //launcher

    public boolean getLauncherIntakeButton() {
        return joysticks[1].getPOVDown();
    }

    public boolean getLauncherReadyButton() {
        return joysticks[3].btn_2.getAsBoolean();
    }

    public boolean getLauncherFireButton() {
        return joysticks[1].getPOVUp();
    }

    public boolean getLauncherStopButton() {
        return joysticks[1].btn_3.getAsBoolean();
    }

    //calibration

    public boolean getCalibrateAngleMotorButton() {
        return joysticks[0].btn_1.getAsBoolean();
    }

    public boolean getCalibrateGyroButton() {
        return joysticks[1].btn_1.getAsBoolean();
    }

    //macros

    public boolean getArmPickupMacroButton() {
        return joysticks[3].btn_3.getAsBoolean();
    }

    public boolean getArmAmpMacroButton() {
        return joysticks[3].btn_5.getAsBoolean();
    }

    public boolean getArmStageStartMacroButton() {
        return joysticks[3].btn_4.getAsBoolean();
    }

    //auton modes

    public boolean getLeftAutonModeButton() {
        return joysticks[0].btn_12.getAsBoolean();
    }

    public boolean getCenterAutonModeButton() {
        return joysticks[0].btn_10.getAsBoolean();
    }

    public boolean getRightAutonModeButton() {
        return joysticks[0].btn_8.getAsBoolean();
    }

    //general

    private Vector2D getJoystickVector(int i) {
        return joysticks[i].getVector();
    }
}
