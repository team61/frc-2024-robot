package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalancingSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ToggleBalancingCommand extends CommandBase {
    private final SwerveDriveSubsystem swervedrive;
    private final BalancingSubsystem balancingSubsystem;

    public ToggleBalancingCommand(SwerveDriveSubsystem sd, BalancingSubsystem balancer) {
        swervedrive = sd;
        balancingSubsystem = balancer;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (balancingSubsystem.isEnabled()) {
            balancingSubsystem.disable();
            swervedrive.disableWheelBreaks();

        } else {
            balancingSubsystem.enable();
            swervedrive.enableWheelBreaks();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
