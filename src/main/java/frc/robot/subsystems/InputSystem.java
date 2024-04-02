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

    public boolean getForcedRotationMode() {
        //return joysticks[1].btn_2.getAsBoolean();
        return false;
    }

    //arm system

    public double getElevatorPower() {
        if (getLauncherOverrideModeButton()) {
            return 0;
        }
        else {
            return getJoystickVector(2).y;
        }
    }

    public double getArmPower() {
        return -getJoystickVector(3).y;
    }

    public boolean getHandPickupButton() {
        return joysticks[3].getPOVDown();
    }

    public boolean getHandReleaseButton() {
        return joysticks[3].getPOVUp() || joysticks[0].getPOVUp();
    }

    // public boolean getBalancerEngageButton() {
    //     return joysticks[2].getPOVUp();
    // }

    // public boolean getBalancerDisengageButton() {
    //     return joysticks[2].getPOVDown();
    // }

    //launcher

    public boolean getLauncherIntakeButton() {
        return joysticks[1].getPOVDown() || joysticks[2].getPOVDown();
    }

    public boolean getLauncherReadyButton() {
        return joysticks[1].btn_2.getAsBoolean() || joysticks[2].btn_2.getAsBoolean();
    }

    public boolean getLauncherFireButton() {
        return joysticks[1].getPOVUp() || joysticks[2].getPOVUp();
    }

    public boolean getLauncherStopButton() {
        return joysticks[1].btn_3.getAsBoolean() || joysticks[2].btn_3.getAsBoolean();
    }

    public boolean getLauncherOverrideModeButton() {
        return joysticks[2].btn_1.getAsBoolean();
    }

    public double getLauncherOverridePower() {
        return getJoystickVector(2).y;
    }

    //calibration

    public boolean getCalibrateAngleMotorButton() {
        return joysticks[0].btn_1.getAsBoolean();
    }

    public boolean getCalibrateGyroButton() {
        return joysticks[1].btn_1.getAsBoolean();
    }

    public boolean getCalibrateArmSystemButton() {
        return joysticks[3].btn_2.getAsBoolean();
    }

    public boolean getCancelCalibrateArmSystemButton() {
        return joysticks[3].btn_1.getAsBoolean();
    }

    //macros

    public boolean getPickupPresetButton() {
        return joysticks[3].btn_3.getAsBoolean();
    }

    public boolean getAmpPresetButton() {
        return joysticks[3].btn_5.getAsBoolean();
    }

    public boolean getHomePresetButton() {
        return joysticks[3].btn_4.getAsBoolean();
    }

    public boolean getStageStartPresetButton() {
        return joysticks[3].btn_6.getAsBoolean();
    }

    public boolean getElevatorOnlyPresetModeButton() {
        return joysticks[2].btn_4.getAsBoolean();
    }

    //auton modes

    public boolean getLeftAutonModeButton() {
        return joysticks[2].btn_12.getAsBoolean() || joysticks[3].btn_12.getAsBoolean();
    }

    public boolean getCenterAutonModeButton() {
        return joysticks[2].btn_10.getAsBoolean() || joysticks[3].btn_10.getAsBoolean();
    }

    public boolean getRightAutonModeButton() {
        return joysticks[2].btn_8.getAsBoolean() || joysticks[3].btn_8.getAsBoolean();
    }

    //leds

    public boolean getSpiritLedButton() {
        return joysticks[0].getThrottle() < -0.9 && joysticks[1].getThrottle() < -0.9 && joysticks[2].getThrottle() < -0.9 && joysticks[3].getThrottle() < -0.9;
    }

    public boolean getLauncherLedButton() {
        return joysticks[2].getThrottle() < -0.9;
    }

    public boolean getAmpLedButton() {
        return joysticks[3].getThrottle() < -0.9;
    }

    //general

    private Vector2D getJoystickVector(int i) {
        return joysticks[i].getVector();
    }
}
