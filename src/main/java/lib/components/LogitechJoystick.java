package lib.components;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Vector2D;

public class LogitechJoystick extends Joystick {

    public LogitechJoystick(int port) {
        super(port);
    }

    public JoystickButton btn_1 = new JoystickButton(this, 1);
    public JoystickButton btn_2 = new JoystickButton(this, 2);
    public JoystickButton btn_3 = new JoystickButton(this, 3);
    public JoystickButton btn_4 = new JoystickButton(this, 4);
    public JoystickButton btn_5 = new JoystickButton(this, 5);
    public JoystickButton btn_6 = new JoystickButton(this, 6);
    public JoystickButton btn_7 = new JoystickButton(this, 7);
    public JoystickButton btn_8 = new JoystickButton(this, 8);
    public JoystickButton btn_9 = new JoystickButton(this, 9);
    public JoystickButton btn_10 = new JoystickButton(this, 10);
    public JoystickButton btn_11 = new JoystickButton(this, 11);
    public JoystickButton btn_12 = new JoystickButton(this, 12);

    public Vector2D getVector() {
        Vector2D vector = new Vector2D(getX(), -getY());

        if (vector.x > Constants.joystickDeadzoneXMin && vector.x < Constants.joystickDeadzoneXMax
        && vector.y > Constants.joystickDeadzoneYMin && vector.y < Constants.joystickDeadzoneYMax) {
            vector = Vector2D.zero;
        }

        return vector;
    }

    // public double getXAxis() {
    //     return getAxis(getX());
    // }

    // public double getXAxis(double buffer) {
    //     return getAxis(getX(), buffer);
    // }

    // public double getYAxis() {
    //     return getAxis(getY());
    // }

    // public double getYAxis(double buffer) {
    //     return getAxis(getY(), buffer);
    // }

    // public double getZAxis() {
    //     return getAxis(getZ());
    // }

    // public double getZAxis(double buffer) {
    //     return getAxis(getZ(), buffer);
    // }

    // private double getAxis(double value, double deadZone) {
    //     if (Math.abs(value) > deadZone) {
    //         return value;
    //     } else {
    //         return 0;
    //     }
    // }
    // private double getAxis(double value) {
    //     return getAxis(value, Constants.joystickDeadzone);
    // }
}