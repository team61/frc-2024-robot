package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  public final SwerveDriveSubsystem swervedrive;
  public final TankDriveSubsystem tankdrive;

  public DriveTrain(SwerveDriveSubsystem sd, TankDriveSubsystem td) {
    swervedrive = sd;
    tankdrive = td;
  }

  public void enableBreaks() {
    swervedrive.enableBreaks();
  }

  public void disableBreaks() {
    swervedrive.disableBreaks();
  }

  @Override
  public void periodic() {}
}
