package frc.robot;

import lib.components.LogitechJoystick;
import static frc.robot.Constants.*;

import frc.robot.commands.TestCommand;

public class RobotContainer {
    public final LogitechJoystick joystick1 = new LogitechJoystick(joystickPort1);
    public final LogitechJoystick joystick2 = new LogitechJoystick(joystickPort2);
    public final LogitechJoystick joystick3 = new LogitechJoystick(joystickPort3);
    public final LogitechJoystick joystick4 = new LogitechJoystick(joystickPort4);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
       joystick1.btn_1.onTrue(new TestCommand());
    }
}
