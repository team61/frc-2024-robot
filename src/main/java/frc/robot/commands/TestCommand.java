package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand extends CommandBase {
    public TestCommand() {}

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println(1);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
