package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroOutArmCommand extends CommandBase {
    private final ArmSubsystem arm;

    public ZeroOutArmCommand(ArmSubsystem a) {
        arm = a;

        addRequirements(a);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.zero();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
