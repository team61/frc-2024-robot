package frc.robot.subsystems;

import lib.components.LogitechJoystick;

import frc.robot.Constants;
import frc.robot.Vector2D;

public class InputSystem {
    private static InputSystem system;

    public LogitechJoystick[] joysticks;

    private InputSystem() {
        joysticks = new LogitechJoystick[Constants.joystickNumbers.length];

        for (int i = 0; i < Constants.joystickNumbers.length; i++) {
            joysticks[i] = new LogitechJoystick(Constants.joystickNumbers[i]);
        }
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

    public double getMainThrottle() {
        return ((-joysticks[0].getThrottle() + 1) / 2) * (Constants.throttleMax - Constants.throttleMin) + Constants.throttleMin;
    }

    public boolean getCallibrateButton() {
        return joysticks[0].btn_1.getAsBoolean();
    }

    private Vector2D getJoystickVector(int i) {
        return new Vector2D(joysticks[i].getXAxis(), -joysticks[i].getYAxis());
    }
}
