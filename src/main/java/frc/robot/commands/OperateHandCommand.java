package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

public class OperateHandCommand extends CommandBase {    
    ArmSystem armSystem = ArmSystem.get();
    
    boolean stop, pickup;

    public OperateHandCommand() {
        stop = true;
    }

    public OperateHandCommand(boolean pickup) {
        stop = false;
        this.pickup = pickup;
    }

    @Override
    public void initialize() {
        if (stop) {
            armSystem.stopHand();
        }
        else if (pickup) {
            armSystem.pickup();
        }
        else {
            armSystem.release();
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
