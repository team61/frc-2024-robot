package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class RotateClawCommand extends CommandBase {
    private final ClawSubsystem claw;

    public RotateClawCommand(ClawSubsystem c) {
        claw = c;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (claw.isRotationUninitialized()) {
            claw.rotateUp();
        } else {
            claw.toggleRotation();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
