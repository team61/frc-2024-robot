package frc.robot;

import lib.components.LogitechJoystick;
import static frc.robot.Constants.*;

import frc.robot.commands.AlignMotorsCommand;
import frc.robot.commands.SetDriveModeCommand;
import frc.robot.commands.ZeroOutMotorsCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class RobotContainer {
    public final LogitechJoystick joystick1 = new LogitechJoystick(joystickPort1);
    public final LogitechJoystick joystick2 = new LogitechJoystick(joystickPort2);
    public final LogitechJoystick joystick3 = new LogitechJoystick(joystickPort3);
    public final LogitechJoystick joystick4 = new LogitechJoystick(joystickPort4);

    public final DriveTrainSubsystem drivetrain = new DriveTrainSubsystem(4, new int[] {
            19, 18,
            11, 10,
            9, 8,
            1, 0,
    });
    public final TankDriveSubsystem tankdrive = new TankDriveSubsystem(2, new int[] {
            19, 18,
            11, 10,
    }, new int[] {
            9, 8,
            1, 0,
    });

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        joystick1.btn_1
                .onTrue(new SetDriveModeCommand(SWERVE_DRIVE))
                .onFalse(new SetDriveModeCommand(TANK_DRIVE));

        joystick2.btn_1
                .whileTrue(new AlignMotorsCommand(drivetrain, MIDDLE));
        joystick2.btn_3
                .or(joystick2.btn_5)
                .whileTrue(new AlignMotorsCommand(drivetrain, FORWARDS));
        joystick2.btn_4
                .or(joystick2.btn_6)
                .whileTrue(new AlignMotorsCommand(drivetrain, SIDEWAYS));
        joystick2.btn_7
                .and(joystick2.btn_8)
                .onTrue(new ZeroOutMotorsCommand(drivetrain));
    }
}
