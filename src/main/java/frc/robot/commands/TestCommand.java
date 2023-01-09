package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand extends CommandBase {
    private boolean finished = false;

    public TestCommand() {}

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println(1);

        end(false);
    }

    @Override
    public void end(boolean interrupted) {
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
