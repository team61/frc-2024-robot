package frc.robot;

import lib.components.LogitechJoystick;
import static frc.robot.Constants.*;

import frc.robot.commands.AlignMotorsCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.SetDriveModeCommand;
import frc.robot.commands.ZeroOutMotorsCommand;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class RobotContainer {
    public final LogitechJoystick joystick1 = new LogitechJoystick(joystickPort1);
    public final LogitechJoystick joystick2 = new LogitechJoystick(joystickPort2);
    public final LogitechJoystick joystick3 = new LogitechJoystick(joystickPort3);
    public final LogitechJoystick joystick4 = new LogitechJoystick(joystickPort4);

    private final SwerveDriveSubsystem swervedrive = new SwerveDriveSubsystem(4, new int[] {
            19, 18,
            11, 10,
            9, 8,
            1, 0,
    });
    private final TankDriveSubsystem tankdrive = new TankDriveSubsystem(2, new int[] {
            19, 18,
            11, 10,
    }, new int[] {
            9, 8,
            1, 0,
    });
    private final DriveTrain drivetrain = new DriveTrain(swervedrive, tankdrive);

    private final DatabaseSubsystem db = new DatabaseSubsystem();

    private final AutonomousCommand autoCommand = new AutonomousCommand(swervedrive, tankdrive);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        joystick1.btn_1
                .onTrue(new SetDriveModeCommand(drivetrain, SWERVE_DRIVE))
                .onFalse(new SetDriveModeCommand(drivetrain, TANK_DRIVE));

        joystick2.btn_1
                .onTrue(new AlignMotorsCommand(swervedrive, MIDDLE));
        joystick2.btn_3
                .or(joystick2.btn_5)
                .onTrue(new AlignMotorsCommand(swervedrive, FORWARDS));
        joystick2.btn_4
                .or(joystick2.btn_6)
                .onTrue(new AlignMotorsCommand(swervedrive, SIDEWAYS));
        joystick2.btn_7
                .and(joystick2.btn_8)
                .onTrue(new ZeroOutMotorsCommand(swervedrive));
    }

    public DriveTrain getDriveTrain() {
        return drivetrain;
    }

    public DatabaseSubsystem getDatabase() {
        return db;
    }

    public AutonomousCommand getAutonomousCommand() {
        return autoCommand;
    }
}
