package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class GrabCommand extends CommandBase {
    private final ClawSubsystem claw;

    public GrabCommand(ClawSubsystem c) {
        claw = c;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (claw.isGrabbingUninitialized()) {
            claw.close();
        } else {
            claw.toggleGrab();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
