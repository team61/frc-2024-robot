package lib.components;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class LogitechJoystick extends Joystick {

    private final double JOYSTICK_BUFFER = 0.1;

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

    public double getAxis(double value, double deadZone) {
        if (Math.abs(value) > deadZone) {
            return value;
        } else {
            return 0;
        }
    }
    public double getAxis(double value) {
        if (Math.abs(value) > JOYSTICK_BUFFER) {
            return value;
        } else {
            return 0;
        }
    }

    public double getYAxis() {
        return getAxis(getY());
    }

    public double getXAxis() {
        return getAxis(getX());
    }

    public double getZAxis(double buffer) { return getAxis(getZ(), buffer); }
}