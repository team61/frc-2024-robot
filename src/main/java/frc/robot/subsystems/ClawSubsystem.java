package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ClawSubsystem extends SubsystemBase {
    private final DoubleSolenoid rotateSolenoid;
    private final DoubleSolenoid grabSolenoid;

    public ClawSubsystem(PneumaticHub hub, int[] solenoid1, int[] solenoid2) {
        rotateSolenoid = hub.makeDoubleSolenoid(solenoid1[0], solenoid1[1]);
        grabSolenoid = hub.makeDoubleSolenoid(solenoid2[0], solenoid2[1]);
    }

    public boolean isRotationUninitialized() {
        return rotateSolenoid.get() == kOff;
    }

    public boolean isGrabbingUninitialized() {
        return grabSolenoid.get() == kOff;
    }

    public Value getPosition() {
        return rotateSolenoid.get();
    }

    public void rotateUp() {
        rotateSolenoid.set(kForward);
    }

    public void rotateDown() {
        rotateSolenoid.set(kReverse);
    }

    public void close() {
        grabSolenoid.set(kForward);
    }

    public void open() {
        grabSolenoid.set(kReverse);
    }

    public void toggleRotation() {
        rotateSolenoid.toggle();
    }

    public void toggleGrab() {
        grabSolenoid.toggle();
    }

    @Override
    public void periodic() {}
}
