package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {
    public static record TargetObservation(
        Rotation2d tx,
        Rotation2d ty
    ) {}

    public static record PoseObservation(
        double timestamp,
        Pose2d pose,
        int tagCount,
        ObservationType type
    ) {}

    public enum ObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTON
    }
}
