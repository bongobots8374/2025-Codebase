package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers;

public class VisionIOReal implements VisionIO{
    public final String name;
    private final NetworkTable table;
    private final DoubleSubscriber latencySubscriber;

    public VisionIOReal(String name){
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
        latencySubscriber = table.getDoubleTopic("t1").subscribe(0.0);

        LimelightHelpers.SetIMUMode(name, 0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.name = name;

        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        
    }
}
