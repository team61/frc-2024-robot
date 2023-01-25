package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class AutonomousCommand extends CommandBase {
    private final SwerveDriveSubsystem swervedrive;
    private final TankDriveSubsystem tankdrive;

    public AutonomousCommand(SwerveDriveSubsystem sd, TankDriveSubsystem td) {
        swervedrive = sd;
        tankdrive = td;

        addRequirements(sd, td);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        new Thread(() -> {
            try {
                tankdrive.driveVolts(4, 4);
                Thread.sleep(2000);
                tankdrive.stop();
                swervedrive.setRotationVoltage(1);
                Thread.sleep(1000);
                swervedrive.setRotationVoltage(0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }).start();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
