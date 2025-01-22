package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private StructPublisher<Pose2d> publisher;
    public boolean useVision = true;

    public Swerve(boolean useVision) throws IOException {
        double maximumSpeed = edu.wpi.first.math.util.Units.feetToMeters(SwerveConstants.MaxSpeed);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);

        this.useVision = useVision;

        publisher = NetworkTableInstance.getDefault()
                .getStructTopic("Pose", Pose2d.struct).publish();

        setupPathPlanner();
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                                DoubleSupplier headingY)
    {
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity(),
                            Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity()),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    true);
        });
    }

    @Override
    public void periodic() {
        if (useVision){
            SmartDashboard.putNumber("Seen Targets Count", LimelightHelpers.getTargetCount("limelight"));
            if (LimelightHelpers.getTargetCount("limelight") > 0){
                visionUpdate();
            }
            swerveDrive.updateOdometry();
        }

        SmartDashboard.putNumber("FL Wheel", swerveDrive.getModules()[0].getAngleMotor().getPosition());
        SmartDashboard.putNumber("FR Wheel", swerveDrive.getModules()[1].getAngleMotor().getPosition());
        SmartDashboard.putNumber("BL Wheel", swerveDrive.getModules()[2].getAngleMotor().getPosition());
        SmartDashboard.putNumber("BR Wheel", swerveDrive.getModules()[3].getAngleMotor().getPosition());

        publisher.set(swerveDrive.getPose());
    }

    private void setupPathPlanner(){
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;

            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward){
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces()
                            );
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)
                    ),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()){
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            );
        } catch (Exception e) {
            e.printStackTrace();
        }

        PathfindingCommand.warmupCommand().schedule();
    }

    private Pose2d getPose(){
        return swerveDrive.getPose();
    }

    private void resetOdometry(Pose2d initialHolonomicPose){
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    private ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    private void visionUpdate(){
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (swerveDrive.getGyro().getYawAngularVelocity().abs(Units.DegreesPerSecond) > 720){
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0){
            doRejectUpdate = true;
        }
        if(!doRejectUpdate){
            swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
        }
    }
}
