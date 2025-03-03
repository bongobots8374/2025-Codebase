package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;

import java.util.function.DoubleSupplier;

public class AlgaeIntake extends SubsystemBase {
    private SparkMax arms;
    private SparkMaxConfig armsConfig;

    public AlgaeIntake() {
        arms = new SparkMax(CanIDs.algaeArmMotor, SparkLowLevel.MotorType.kBrushless);

        armsConfig = new SparkMaxConfig();
        armsConfig
                .inverted(Constants.ElevatorMotorConfig.oneInverted)
                .idleMode(Constants.ElevatorMotorConfig.idleMode);
        armsConfig.encoder
                .positionConversionFactor(Constants.ElevatorMotorConfig.positionConversionFactor)
                .velocityConversionFactor(Constants.ElevatorMotorConfig.velocityConversionFactor);
        armsConfig.closedLoop
                .feedbackSensor(Constants.ElevatorMotorConfig.feedbackSensor)
                .pid(Constants.ElevatorMotorConfig.p, Constants.ElevatorMotorConfig.i, Constants.ElevatorMotorConfig.d);

        arms.configure(armsConfig, Constants.ElevatorMotorConfig.resetMode, Constants.ElevatorMotorConfig.persistMode);
    }

    public Command angleMove(DoubleSupplier input) {
        return run(() -> {
            arms.set(input.getAsDouble());
        });
    }
}
