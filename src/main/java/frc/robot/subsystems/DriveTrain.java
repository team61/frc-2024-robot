package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  public final OldSwerveDriveSubsystem swervedrive;
  public final TankDriveSubsystem tankdrive;

  public DriveTrain(OldSwerveDriveSubsystem sd, TankDriveSubsystem td) {
    swervedrive = sd;
    tankdrive = td;
  }

  // public void enableBreaks() {
  //   swervedrive.enableBreaks();
  // }

  // public void disableBreaks() {
  //   swervedrive.disableBreaks();
  // }

  public void enableWheelBreaks() {
    swervedrive.enableWheelBreaks();
  }

  public void disableWheelBreaks() {
    swervedrive.disableWheelBreaks();
  }

  @Override
  public void periodic() {}
}
