package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

public class GrabGamePieceCommand extends CommandBase {
    private final String piece;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private boolean finished = false;
    public GrabGamePieceCommand(String p, ElevatorSubsystem e, ArmSubsystem a, ClawSubsystem c) {
        piece = p;
        elevator = e;
        arm = a;
        claw = c;

        addRequirements(e, a, c);
    }

    @Override
    public void initialize() {
        claw.rotateUp();
    }

    @Override
    public void execute() {
        double elevatorPos = elevator.getPosition();
        double armPos = arm.getPosition();
        double elevatorVolts = 0;
        double armVolts = 0;
        if (piece == BLOCK) {
            if (Math.abs(elevatorPos - ELEVATOR_GRAB_BLOCK_POSITION) > 5000) {
                elevatorVolts = -Math.signum(elevatorPos - ELEVATOR_GRAB_BLOCK_POSITION) * 8;
            } else if (Math.abs(armPos - ARM_GRAB_BLOCK_POSITION) > 500) {
                armVolts = -Math.signum(armPos - ARM_GRAB_BLOCK_POSITION) * 2;
            } else {
                finished = true;
            }
        }

        elevator.setVoltage(arm, elevatorVolts);
        arm.setVoltage(elevator, armVolts);
    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
