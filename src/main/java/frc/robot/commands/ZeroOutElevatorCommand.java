package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroOutElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;

    public ZeroOutElevatorCommand(ElevatorSubsystem e) {
        elevator = e;

        addRequirements(e);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.zero();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
