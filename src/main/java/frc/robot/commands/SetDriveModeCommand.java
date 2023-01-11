package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Globals.*;

public class SetDriveModeCommand extends CommandBase {
    private final String mode;
    private boolean finished = false;

    public SetDriveModeCommand(String newMode) {
        mode = newMode;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        CURRENT_DRIVE_MODE = mode;

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
