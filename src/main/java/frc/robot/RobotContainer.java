package frc.robot;

import lib.components.LogitechJoystick;
import static frc.robot.Constants.*;
import static frc.robot.Globals.*;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.GrabGamePieceCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.RotateClawCommand;
import frc.robot.commands.SetDriveModeCommand;
import frc.robot.commands.ToggleBalancingCommand;
import frc.robot.commands.ZeroOutArmCommand;
import frc.robot.commands.ZeroOutElevatorCommand;
import frc.robot.commands.ZeroYawCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.OldSwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class RobotContainer {
	public final LogitechJoystick joystick1 = new LogitechJoystick(joystickPort1);
	public final LogitechJoystick joystick2 = new LogitechJoystick(joystickPort2);
	public final LogitechJoystick joystick3 = new LogitechJoystick(joystickPort3);
	public final LogitechJoystick joystick4 = new LogitechJoystick(joystickPort4);

	private final OldSwerveDriveSubsystem swervedrive = new OldSwerveDriveSubsystem(4, new int[] {
			2, 3,
			0, 1,
			16, 17,
			18, 19,
	}, new int[] {
			22,
			23,
			24,
			21,
	}, new double[] {
			0,0,0,0
	});
	private final TankDriveSubsystem tankdrive = new TankDriveSubsystem(2, new int[] {
			11, 10,
			19, 18,
	}, new int[] {
			1, 0,
			9, 8,
	});

	public final PneumaticHub pneumaticHub = new PneumaticHub(51);
	public final ElevatorSubsystem elevator = new ElevatorSubsystem(2);
	public final ArmSubsystem arm = new ArmSubsystem(3, 0);
	private final ClawSubsystem claw = new ClawSubsystem(pneumaticHub, new int[] { 0, 1 }, new int[] { 2, 3 });
	private final LEDStripSubsystem ledStrip = new LEDStripSubsystem(0, 56);

	private final AHRS gyro = new AHRS(Port.kMXP);
	private final DriveTrain drivetrain = new DriveTrain(swervedrive, tankdrive);
	public final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem(gyro);
	private final BalancingSubsystem balancingSubsystem = new BalancingSubsystem(gyro, swerve);
	
	private final AutonomousCommand autoCommand = new AutonomousCommand(swerve, drivetrain, gyro, elevator, arm, claw, balancingSubsystem);

	public RobotContainer() {
		swerve.setDefaultCommand(
            new DriveCommand(
                swerve, 
                (DoubleSupplier)() -> -joystick1.getYAxis(), 
                (DoubleSupplier)() -> -joystick1.getXAxis(), 
                (DoubleSupplier)() -> -joystick2.getXAxis(),
				joystick2.btn_2,
                joystick1.btn_2));

		elevator.setDefaultCommand(
			new MoveElevatorCommand(
				elevator,
				arm,
				(DoubleSupplier)() -> joystick3.getYAxis(0.15),
				joystick3.btn_2));
			
		arm.setDefaultCommand(
			new MoveArmCommand(
				arm,
				elevator,
				(DoubleSupplier)() -> joystick4.getYAxis(0.20),
				joystick4.btn_2));
		
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		joystick1.btn_1
				.onTrue(new SetDriveModeCommand(TANK_DRIVE))
				.onFalse(new SetDriveModeCommand(SWERVE_DRIVE));
		joystick1.btn_3
			.or(joystick1.btn_5)
			.onTrue(new InstantCommand(() -> {
				drivetrain.enableWheelBreaks();
			}));
		joystick1.btn_4
			.or(joystick1.btn_6)
			.onTrue(new InstantCommand(() -> {
				drivetrain.disableWheelBreaks();
			}));
		// joystick1.btn_12
		// 		.onTrue(new ToggleBalancingCommand(swervedrive, balancingSubsystem));

		joystick2.btn_7
				.and(joystick2.btn_8)
				.onTrue(new ZeroYawCommand(gyro));
		joystick2.btn_12
				.onTrue(new InstantCommand(() -> {
					swerve.mSwerveMods[0].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[0].getAngle().getDegrees());
					swerve.mSwerveMods[1].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[1].getAngle().getDegrees());
					swerve.mSwerveMods[2].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[2].getAngle().getDegrees());
					swerve.mSwerveMods[3].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[3].getAngle().getDegrees());
					// swerve.mSwerveMods[0].angleEncoder.configMagnetOffset(0.0);
					// swerve.mSwerveMods[1].angleEncoder.configMagnetOffset(0.0);
					// swerve.mSwerveMods[2].angleEncoder.configMagnetOffset(0.0);
					// swerve.mSwerveMods[3].angleEncoder.configMagnetOffset(0.0);
				}));
				
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
				.onTrue(new InstantCommand(() -> { IS_RECORDING = !IS_RECORDING; }));
	}

	public DriveTrain getDriveTrain() {
		return drivetrain;
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

	public Command getAutonomousCommand() {
		return autoCommand;
	}
}
