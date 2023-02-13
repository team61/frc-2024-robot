package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalancingSubsystem;

public class ToggleBalancingCommand extends CommandBase {
    private final BalancingSubsystem balancingSubsystem;

    public ToggleBalancingCommand(BalancingSubsystem subsystem) {
        balancingSubsystem = subsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (balancingSubsystem.isEnabled()) {
            balancingSubsystem.disable();
        } else {
            balancingSubsystem.enable();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
