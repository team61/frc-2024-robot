package frc.robot.subsystems;

import lib.components.LogitechJoystick;

import frc.robot.Constants;
import frc.robot.Vector2D;

public class InputSystem {
    private static InputSystem system;

    public LogitechJoystick[] joysticks;

    private int throttlePreset;

    private InputSystem() {
        joysticks = new LogitechJoystick[Constants.joystickNumbers.length];

        for (int i = 0; i < Constants.joystickNumbers.length; i++) {
            joysticks[i] = new LogitechJoystick(Constants.joystickNumbers[i]);
        }

        throttlePreset = Constants.defaultThrottlePreset;
    }

    public static InputSystem get() {
        if (system == null) {
            system = new InputSystem();
        }

        return system;
    }

    public Vector2D getTranslationVector() {
        return getJoystickVector(0);
    }

    public double getRotationPowerLinear() {
        return joysticks[1].getZ();
    }

    public Vector2D getTargetAngleVector() {
        return joysticks[1].getVector();
    }

    public void updateThrottle() {
        if (joysticks[0].btn_12.getAsBoolean()) {
            throttlePreset = 0;
        }
        else if (joysticks[0].btn_10.getAsBoolean()) {
            throttlePreset = 1;
        }
        else if (joysticks[0].btn_8.getAsBoolean()) {
            throttlePreset = 2;
        }
        else if (joysticks[0].btn_11.getAsBoolean()) {
            throttlePreset = 3;
        }
        else if (joysticks[0].btn_9.getAsBoolean()) {
            //throttlePreset = 4;
        }
        else if (joysticks[0].btn_7.getAsBoolean()) {
            //throttlePreset = 5;
        }
    }

    public double getTranslationThrottle() {
        return Constants.throttlePresets[throttlePreset].translationFactor;
    }

    public double getRotationThrottle() {
        return Constants.throttlePresets[throttlePreset].rotationFactor;
    }

    public boolean getLinearRotationMode() {
        return joysticks[1].btn_2.getAsBoolean();
    }

    public double getElevatorPower() {
        return getJoystickVector(2).y;
    }

    public double getArmPower() {
        return -getJoystickVector(3).y;
    }

    public boolean getHandPickupButton() {
        return joysticks[3].btn_3.getAsBoolean();
    }

    public boolean getHandReleaseButton() {
        return joysticks[3].btn_5.getAsBoolean();
    }

    public boolean getCallibrateButton() {
        return joysticks[0].btn_1.getAsBoolean();
    }

    public boolean getResetGyroButton() {
        return joysticks[1].btn_1.getAsBoolean();
    }

    public boolean getLauncherIntakeButton() {
        return joysticks[2].btn_4.getAsBoolean();
    }

    public boolean getLauncherReadyButton() {
        return joysticks[2].btn_3.getAsBoolean();
    }

    public boolean getLauncherFireButton() {
        return joysticks[2].btn_5.getAsBoolean();
    }

    public boolean getLauncherCancelButton() {
        return joysticks[2].btn_6.getAsBoolean();
    }

    private Vector2D getJoystickVector(int i) {
        return joysticks[i].getVector();
    }
}
