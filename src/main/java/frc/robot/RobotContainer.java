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
import frc.robot.commands.ZeroYawCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
	public final LogitechJoystick joystick1 = new LogitechJoystick(joystickPort1);
	public final LogitechJoystick joystick2 = new LogitechJoystick(joystickPort2);
	public final LogitechJoystick joystick3 = new LogitechJoystick(joystickPort3);
	public final LogitechJoystick joystick4 = new LogitechJoystick(joystickPort4);

	

	public final PneumaticHub pneumaticHub = new PneumaticHub(51);

	private final AHRS gyro = new AHRS(Port.kMXP);
	public final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem(gyro);
	
	private final AutonomousCommand autoCommand = new AutonomousCommand(swerve, gyro);

	public RobotContainer() {
		swerve.setDefaultCommand(
            new DriveCommand(
                swerve, 
                (DoubleSupplier)() -> -joystick1.getYAxis(), 
                (DoubleSupplier)() -> -joystick1.getXAxis(), 
                (DoubleSupplier)() -> -joystick2.getXAxis(),
				joystick2.btn_2,
                joystick1.btn_2));

		
		
		// configureButtonBindings();
	}

	private void configureButtonBindings() {

		// joystick1.btn_1
		// 		.onTrue(new SetDriveModeCommand(TANK_DRIVE))
		// 		.onFalse(new SetDriveModeCommand(SWERVE_DRIVE));

		// joystick1.btn_1
		// 		.onTrue(new SetDriveModeCommand(TANK_DRIVE))
		// 		.onFalse(new SetDriveModeCommand(SWERVE_DRIVE));

// 		joystick1.btn_3
// 			.or(joystick1.btn_5)
// 			.onTrue(new InstantCommand(() -> {
// 				drivetrain.enableWheelBreaks();
// 			}));
// 		joystick1.btn_4
// 			.or(joystick1.btn_6)
// 			.onTrue(new InstantCommand(() -> {
// 				drivetrain.disableWheelBreaks();
// 			}));
		// joystick1.btn_12
		// 		.onTrue(new ToggleBalancingCommand(swervedrive, balancingSubsystem));

	// 	joystick2.btn_7
	// 			.and(joystick2.btn_8)
	// 			.onTrue(new ZeroYawCommand(gyro));
	// 	joystick2.btn_12
	// 			.onTrue(new InstantCommand(() -> {
	// 				swerve.mSwerveMods[0].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[0].getAngle().getDegrees());
	// 				swerve.mSwerveMods[1].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[1].getAngle().getDegrees());
	// 				swerve.mSwerveMods[2].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[2].getAngle().getDegrees());
	// 				swerve.mSwerveMods[3].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[3].getAngle().getDegrees());
	// 				// swerve.mSwerveMods[0].angleEncoder.configMagnetOffset(0.0);
	// 				// swerve.mSwerveMods[1].angleEncoder.configMagnetOffset(0.0);
	// 				// swerve.mSwerveMods[2].angleEncoder.configMagnetOffset(0.0);
	// 				// swerve.mSwerveMods[3].angleEncoder.configMagnetOffset(0.0);
	// 			}));
				
	// 	joystick3.btn_1
	// 			.onTrue(new RotateClawCommand(claw));
	// 	joystick3.btn_7
	// 			.and(joystick3.btn_8)
	// 			.onTrue(new ZeroOutElevatorCommand(elevator));
	// 	joystick3.btn_11
	// 			.onTrue(new GrabGamePieceCommand(HUMAN, elevator, arm, claw));
	// 	joystick3.btn_12
	// 			.onTrue(new GrabGamePieceCommand(FLOOR, elevator, arm, claw));

	// 	joystick4.btn_1
	// 			.onTrue(new GrabCommand(claw));
	// 	joystick4.btn_7
	// 			.and(joystick4.btn_8)
	// 			.onTrue(new ZeroOutArmCommand(arm));
	// 	joystick4.btn_11
	// 			.and(joystick4.btn_12)
	// 			.onTrue(new InstantCommand(() -> { IS_RECORDING = !IS_RECORDING; }));
	// }

	// public DriveTrain getDriveTrain() {
	// 	return drivetrain;
	}

		// joystick1.btn_3
		// 	.or(joystick1.btn_5)
		// 	.onTrue(new InstantCommand(() -> {
		// 		drivetrain.enableWheelBreaks();
		// 	}));
		// joystick1.btn_4
		// 	.or(joystick1.btn_6)
		// 	.onTrue(new InstantCommand(() -> {
		// 		drivetrain.disableWheelBreaks();
		// 	}));
		// joystick1.btn_12
		// 		.onTrue(new ToggleBalancingCommand(swervedrive, balancingSubsystem));

		// joystick2.btn_7
		// 		.and(joystick2.btn_8)
		// 		.onTrue(new ZeroYawCommand(gyro));
		// joystick2.btn_12
		// 		.onTrue(new InstantCommand(() -> {
		// 			swerve.mSwerveMods[0].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[0].getAngle().getDegrees());
		// 			swerve.mSwerveMods[1].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[1].getAngle().getDegrees());
		// 			swerve.mSwerveMods[2].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[2].getAngle().getDegrees());
		// 			swerve.mSwerveMods[3].angleEncoder.configMagnetOffset(-swerve.mSwerveMods[3].getAngle().getDegrees());
					// swerve.mSwerveMods[0].angleEncoder.configMagnetOffset(0.0);
					// swerve.mSwerveMods[1].angleEncoder.configMagnetOffset(0.0);
					// swerve.mSwerveMods[2].angleEncoder.configMagnetOffset(0.0);
					// swerve.mSwerveMods[3].angleEncoder.configMagnetOffset(0.0);
				// }));
		
	


	public AHRS getGyro() {
		return gyro;
	}

	public Command getAutonomousCommand() {
		return autoCommand;
	}
}
