package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

public class SimCamera {
    public final VisionSystemSim system;
    private final SimCameraProperties properties;
    public final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;

    private AprilTagFieldLayout layout;

    public final String name;

    public SimCamera(String name, Transform3d robotToCamera){
        this.name = name;

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (IOException ex){
            DriverStation.reportError("THE APRIL TAG FIELD IS FAKE", ex.getStackTrace());
        }

        system = new VisionSystemSim(name);

        properties = new SimCameraProperties();

        properties.setCalibError(0.25, 0.08);
        properties.setRandomSeed((long) 0.011);

        camera = new PhotonCamera(name);
        cameraSim = new PhotonCameraSim(camera, properties);

        cameraSim.enableProcessedStream(true);
        cameraSim.enableRawStream(true);
        cameraSim.enableDrawWireframe(true);

        system.addCamera(cameraSim, robotToCamera);

        system.addAprilTags(layout);

        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d previous, PhotonPipelineResult result){
        poseEstimator.setReferencePose(previous);
        return poseEstimator.update(result);
    }
}
