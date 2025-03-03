package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralMotorConfig;
import frc.robot.Constants.CanIDs;

import java.util.function.DoubleSupplier;

public class CoralIntake extends SubsystemBase {
    private SparkMax angle;
    private SparkMax intakeLeft;
    private SparkMax intakeRight;
    private SparkMaxConfig angleConfig;
    private SparkMaxConfig intakeLeftConfig;
    private SparkMaxConfig intakeRightConfig;

    public CoralIntake() {
        angle = new SparkMax(CanIDs.coralAngleMotor, SparkLowLevel.MotorType.kBrushless);
        intakeLeft = new SparkMax(CanIDs.leftCoralIntakeMotor, SparkLowLevel.MotorType.kBrushless);
        intakeRight = new SparkMax(CanIDs.rightCoralIntakeMotor, SparkLowLevel.MotorType.kBrushless);

        angleConfig = new SparkMaxConfig();
        angleConfig
                .inverted(CoralMotorConfig.angleInverted)
                .idleMode(CoralMotorConfig.angleIdleMode);
        angleConfig.encoder
                .positionConversionFactor(CoralMotorConfig.positionConversionFactor)
                .velocityConversionFactor(CoralMotorConfig.velocityConversionFactor);
        angleConfig.closedLoop
                .feedbackSensor(CoralMotorConfig.feedbackSensor)
                .pid(CoralMotorConfig.p, CoralMotorConfig.i, CoralMotorConfig.d);

        intakeLeftConfig = new SparkMaxConfig();
        intakeLeftConfig
                .inverted(CoralMotorConfig.leftInverted)
                .idleMode(CoralMotorConfig.intakeMode);
        intakeLeftConfig.encoder
                .positionConversionFactor(CoralMotorConfig.positionConversionFactor)
                .velocityConversionFactor(CoralMotorConfig.velocityConversionFactor);
        intakeLeftConfig.closedLoop
                .feedbackSensor(CoralMotorConfig.feedbackSensor)
                .pid(CoralMotorConfig.p, CoralMotorConfig.i, CoralMotorConfig.d);

        intakeRightConfig = new SparkMaxConfig();
        intakeRightConfig
                .inverted(CoralMotorConfig.rightInverted)
                .idleMode(CoralMotorConfig.intakeMode)
                .follow(CanIDs.leftCoralIntakeMotor);
        intakeRightConfig.encoder
                .positionConversionFactor(CoralMotorConfig.positionConversionFactor)
                .velocityConversionFactor(CoralMotorConfig.velocityConversionFactor);
        intakeRightConfig.closedLoop
                .feedbackSensor(CoralMotorConfig.feedbackSensor)
                .pid(CoralMotorConfig.p, CoralMotorConfig.i, CoralMotorConfig.d);

        angle.configure(angleConfig, CoralMotorConfig.resetMode, CoralMotorConfig.persistMode);
        intakeLeft.configure(intakeLeftConfig, CoralMotorConfig.resetMode, CoralMotorConfig.persistMode);
        intakeRight.configure(intakeRightConfig, CoralMotorConfig.resetMode, CoralMotorConfig.persistMode);
    }

    public Command move(DoubleSupplier angleSpeed, DoubleSupplier intake){
        return run(() -> {
            angle.set(angleSpeed.getAsDouble());
            intakeLeft.set(intake.getAsDouble());
        });
    }

    /*
    public Command angleMove(DoubleSupplier input) {
        return run(() -> {
            angle.set(input.getAsDouble());
        });
    }

    public Command intake(DoubleSupplier input) {
        return run(() -> {
            intakeLeft.set(input.getAsDouble());
        });
    }
     */
}
