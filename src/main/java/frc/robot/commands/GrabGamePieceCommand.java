package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

public class GrabGamePieceCommand extends CommandBase {
    private final String position;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private boolean finished = false;
    public GrabGamePieceCommand(String p, ElevatorSubsystem e, ArmSubsystem a, ClawSubsystem c) {
        position = p;
        elevator = e;
        arm = a;
        claw = c;

        addRequirements(e, a, c);
    }

    @Override
    public void initialize() {
        if (position.equals(HUMAN)) {
            claw.rotateUp();
        }
    }

    @Override
    public void execute() {
        double elevatorPos = elevator.getPosition();
        double armPos = arm.getPosition();
        double targetElevatorPos = position.equals(HUMAN) ? ELEVATOR_HUMAN_PIECE_POSITION : ELEVATOR_FLOOR_PIECE_POSITION;
        double targetArmPos = position.equals(HUMAN) ? ARM_HUMAN_PIECE_POSITION : ARM_FLOOR_PIECE_POSITION;
        double elevatorVolts = 0;
        double armVolts = 0;

        if (Math.abs(elevatorPos - targetElevatorPos) > 5000) {
            elevatorVolts = -Math.signum(elevatorPos - targetElevatorPos) * 8;
        }
        
        if (Math.abs(armPos - targetArmPos) > 500) {
            armVolts = -Math.signum(armPos - targetArmPos) * 2;
        }

        if (elevatorVolts == 0 && armVolts == 0) {
            finished = true;
        }

        elevator.setVoltageUnsafe(elevatorVolts);
        arm.setVoltageUnsafe(armVolts);
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
