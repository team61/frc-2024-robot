package frc.robot.subsystems;

import lib.components.LogitechJoystick;

import frc.robot.Constants;
import frc.robot.Vector2D;

public class InputSystem {
    private static InputSystem system;

    public LogitechJoystick[] joysticks;

    private double throttle;

    private InputSystem() {
        joysticks = new LogitechJoystick[Constants.joystickNumbers.length];

        for (int i = 0; i < Constants.joystickNumbers.length; i++) {
            joysticks[i] = new LogitechJoystick(Constants.joystickNumbers[i]);
        }

        throttle = Constants.throttlePresets[Constants.defaultThrottlePreset];
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

    public double getRotationPower() {
        return joysticks[1].getVector().y;
    }

    public Vector2D getTargetAngleVector() {
        return joysticks[1].getVector();
    }

    public void updateMainThrottle() {
        if (joysticks[0].btn_12.getAsBoolean()) {
            throttle = Constants.throttlePresets[0];
        }
        else if (joysticks[0].btn_10.getAsBoolean()) {
            throttle = Constants.throttlePresets[1];
        }
        else if (joysticks[0].btn_8.getAsBoolean()) {
            throttle = Constants.throttlePresets[2];
        }
    }

    public double getMainThrottle() {
        return throttle;
    }

    public boolean getCallibrateButton() {
        return joysticks[0].btn_1.getAsBoolean();
    }

    public boolean getResetGyroButton() {
        return joysticks[1].btn_1.getAsBoolean();
    }

    private Vector2D getJoystickVector(int i) {
        return joysticks[i].getVector();
    }
}
