package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DatabaseSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final IntegerArraySubscriber tagsSub;

    public DatabaseSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("bvtrobotics");
        tagsSub = table.getIntegerArrayTopic("found_tags").subscribe(new long[] {});
    }

    public long[] getTags() {
        return tagsSub.get();
    }

    @Override
    public void periodic() {}

    @Override public void simulationPeriodic() {}
}
