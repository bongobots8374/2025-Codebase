package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeMotorConfig;
import frc.robot.Constants.CanIDs;

import java.util.function.DoubleSupplier;

public class AlgaeIntake extends SubsystemBase {
    private SparkMax arms;
    private SparkMax intakeLeft;
    private SparkMax intakeRight;
    private SparkMaxConfig armsConfig;
    private SparkMaxConfig intakeLeftConfig;
    private SparkMaxConfig intakeRightConfig;

    public AlgaeIntake() {
        arms = new SparkMax(CanIDs.algaeArmMotor, SparkLowLevel.MotorType.kBrushless);
        intakeLeft = new SparkMax(CanIDs.leftAlgaeIntakeMotor, SparkLowLevel.MotorType.kBrushless);
        intakeRight = new SparkMax(CanIDs.rightAlgaeIntakeMotor, SparkLowLevel.MotorType.kBrushless);

        armsConfig = new SparkMaxConfig();
        armsConfig
                .inverted(AlgaeMotorConfig.armsInverted)
                .idleMode(AlgaeMotorConfig.armsIdleMode);
        armsConfig.encoder
                .positionConversionFactor(AlgaeMotorConfig.positionConversionFactor)
                .velocityConversionFactor(AlgaeMotorConfig.velocityConversionFactor);
        armsConfig.closedLoop
                .feedbackSensor(AlgaeMotorConfig.feedbackSensor)
                .pid(AlgaeMotorConfig.p, AlgaeMotorConfig.i, AlgaeMotorConfig.d);

        intakeLeftConfig = new SparkMaxConfig();
        intakeLeftConfig
                .inverted(AlgaeMotorConfig.leftInverted)
                .idleMode(AlgaeMotorConfig.intakeMode);
        intakeLeftConfig.encoder
                .positionConversionFactor(AlgaeMotorConfig.positionConversionFactor)
                .velocityConversionFactor(AlgaeMotorConfig.velocityConversionFactor);
        intakeLeftConfig.closedLoop
                .feedbackSensor(AlgaeMotorConfig.feedbackSensor)
                .pid(AlgaeMotorConfig.p, AlgaeMotorConfig.i, AlgaeMotorConfig.d);

        intakeRightConfig = new SparkMaxConfig();
        intakeRightConfig
                .inverted(AlgaeMotorConfig.rightInverted)
                .idleMode(AlgaeMotorConfig.intakeMode)
                .follow(CanIDs.leftAlgaeIntakeMotor);
        intakeRightConfig.encoder
                .positionConversionFactor(AlgaeMotorConfig.positionConversionFactor)
                .velocityConversionFactor(AlgaeMotorConfig.velocityConversionFactor);
        intakeRightConfig.closedLoop
                .feedbackSensor(AlgaeMotorConfig.feedbackSensor)
                .pid(AlgaeMotorConfig.p, AlgaeMotorConfig.i, AlgaeMotorConfig.d);

        arms.configure(armsConfig, AlgaeMotorConfig.resetMode, AlgaeMotorConfig.persistMode);
        intakeLeft.configure(intakeLeftConfig, AlgaeMotorConfig.resetMode, AlgaeMotorConfig.persistMode);
        intakeRight.configure(intakeRightConfig, AlgaeMotorConfig.resetMode, AlgaeMotorConfig.persistMode);
    }

    public Command angleMove(DoubleSupplier input) {
        return run(() -> {
            arms.set(input.getAsDouble());
        });
    }

    public Command intake(DoubleSupplier input) {
        return run(() -> {
            intakeLeft.set(input.getAsDouble());
        });
    }
}
