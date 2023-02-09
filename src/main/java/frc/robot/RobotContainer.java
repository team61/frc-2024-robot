package frc.robot;

import lib.components.LogitechJoystick;
import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignMotorsCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.IndividualWheelRotationCommand;
import frc.robot.commands.SetDriveModeCommand;
import frc.robot.commands.ZeroOutMotorsCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
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
    }, new int[] {
            30,
            31,
            32,
            33,
    });
    private final TankDriveSubsystem tankdrive = new TankDriveSubsystem(2, new int[] {
        11, 10,
        19, 18,
    }, new int[] {
            1, 0,
            9, 8,
    });
    private final DriveTrain drivetrain = new DriveTrain(swervedrive, tankdrive);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(2);
    private final ArmSubsystem arm = new ArmSubsystem(3);
    private final AHRS gyro = new AHRS(Port.kMXP);

    public final PneumaticHub pneumaticHub = new PneumaticHub(51);

    private final DatabaseSubsystem db = new DatabaseSubsystem();

    private final AutonomousCommand autoCommand = new AutonomousCommand(drivetrain, db);

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
        joystick2.btn_9
                .onTrue(new IndividualWheelRotationCommand(swervedrive, 0, 1))
                .onFalse(new IndividualWheelRotationCommand(swervedrive, 0, 0));
        joystick2.btn_10
                .onTrue(new IndividualWheelRotationCommand(swervedrive, 1, 1))
                .onFalse(new IndividualWheelRotationCommand(swervedrive, 1, 0));
        joystick2.btn_11
                .onTrue(new IndividualWheelRotationCommand(swervedrive, 2, 1))
                .onFalse(new IndividualWheelRotationCommand(swervedrive, 2, 0));
        joystick2.btn_12
                .onTrue(new IndividualWheelRotationCommand(swervedrive, 3, 1))
                .onFalse(new IndividualWheelRotationCommand(swervedrive, 3, 0));  
    }

    public DriveTrain getDriveTrain() {
        return drivetrain;
    }

    public ElevatorSubsystem getElevator() {
        return elevator;
    }

    public ArmSubsystem getArm() {
        return arm;
    }

    public AHRS getGyro() {
        return gyro;
    }

    public DatabaseSubsystem getDatabase() {
        return db;
    }

    public SequentialCommandGroup getAutonomousCommand() {
        return autoCommand.andThen(new AlignMotorsCommand(swervedrive, FORWARDS, 2000));
    }
}
