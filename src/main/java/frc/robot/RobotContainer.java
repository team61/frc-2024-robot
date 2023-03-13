package frc.robot;

import lib.components.LogitechJoystick;
import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AlignMotorsCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.GrabGamePieceCommand;
import frc.robot.commands.IndividualWheelRotationCommand;
import frc.robot.commands.RotateClawCommand;
import frc.robot.commands.SetDriveModeCommand;
import frc.robot.commands.ToggleBalancingCommand;
import frc.robot.commands.ZeroOutArmCommand;
import frc.robot.commands.ZeroOutElevatorCommand;
import frc.robot.commands.ZeroOutMotorsCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DatabaseSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
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
	}, new double[] {
			162.176,
			137.375,
			36.858,
			-27.549,
	});
	private final TankDriveSubsystem tankdrive = new TankDriveSubsystem(2, new int[] {
			11, 10,
			19, 18,
	}, new int[] {
			1, 0,
			9, 8,
	});

	public final PneumaticHub pneumaticHub = new PneumaticHub(51);
	private final ElevatorSubsystem elevator = new ElevatorSubsystem(2);
	private final ArmSubsystem arm = new ArmSubsystem(3, 0);
	private final ClawSubsystem claw = new ClawSubsystem(pneumaticHub, new int[] { 0, 1 }, new int[] { 2, 3 });
	private final LEDStripSubsystem ledStrip = new LEDStripSubsystem(0, 56);

	private final DriveTrain drivetrain = new DriveTrain(swervedrive, tankdrive);
	private final AHRS gyro = new AHRS(Port.kMXP);
	private final BalancingSubsystem balancingSubsystem = new BalancingSubsystem(gyro, drivetrain);
	
	private final DatabaseSubsystem db = new DatabaseSubsystem();

	private final AutonomousCommand autoCommand = new AutonomousCommand(drivetrain, gyro, elevator, arm, claw, balancingSubsystem);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		joystick1.btn_1
				.onTrue(new SetDriveModeCommand(drivetrain, TANK_DRIVE))
				.onFalse(new SetDriveModeCommand(drivetrain, SWERVE_DRIVE));
		joystick1.btn_2
				.onTrue(new ToggleBalancingCommand(swervedrive, balancingSubsystem));

		// joystick2.btn_1
		// 		.onTrue(new ZeroYawCommand(gyro));
		joystick2.btn_1
				.onTrue(new AlignMotorsCommand(swervedrive, DIAGONAL));
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
				
		joystick3.btn_1
				.onTrue(new RotateClawCommand(claw));
		joystick3.btn_7
				.and(joystick3.btn_8)
				.onTrue(new ZeroOutElevatorCommand(elevator));
		joystick3.btn_11
				.onTrue(new GrabGamePieceCommand(HUMAN, elevator, arm, claw));
		joystick3.btn_12
				.onTrue(new GrabGamePieceCommand(FLOOR, elevator, arm, claw));

		joystick4.btn_1
				.onTrue(new GrabCommand(claw));
		joystick4.btn_7
				.and(joystick4.btn_8)
				.onTrue(new ZeroOutArmCommand(arm));
		joystick4.btn_11
				.and(joystick4.btn_12)
				.onTrue(new InstantCommand(() -> { IS_RECORDING = true; }));
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

	public ClawSubsystem getClaw() {
		return claw;
	}

	public AHRS getGyro() {
		return gyro;
	}
	
	public BalancingSubsystem getBalancer() {
		return balancingSubsystem;
	}

	public LEDStripSubsystem getLEDStrip() {
		return ledStrip;
	}

	public DatabaseSubsystem getDatabase() {
		return db;
	}

	public Command getAutonomousCommand() {
		return autoCommand;
	}
}
