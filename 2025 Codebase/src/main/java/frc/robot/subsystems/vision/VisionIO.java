package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.PoseObservation;

public interface VisionIO {
    public class VisionIOInputs {
        public String name;
        public boolean connected;
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
