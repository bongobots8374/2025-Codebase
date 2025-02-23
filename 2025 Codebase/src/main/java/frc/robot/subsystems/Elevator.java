package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorMotorConfig;
import frc.robot.Constants.CanIDs;

import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    private SparkMax stageOne;
    private SparkMax stageTwo;
    private SparkMaxConfig oneConfig;
    private SparkMaxConfig twoConfig;

    public Elevator() {
        stageOne = new SparkMax(CanIDs.stageOneMotor, MotorType.kBrushless);
        stageTwo = new SparkMax(CanIDs.stageTwoMotor, MotorType.kBrushless);

        oneConfig = new SparkMaxConfig();
        oneConfig
                .inverted(ElevatorMotorConfig.oneInverted)
                .idleMode(ElevatorMotorConfig.idleMode);
        oneConfig.encoder
                .positionConversionFactor(ElevatorMotorConfig.positionConversionFactor)
                .velocityConversionFactor(ElevatorMotorConfig.velocityConversionFactor);
        oneConfig.closedLoop
                .feedbackSensor(ElevatorMotorConfig.feedbackSensor)
                .pid(ElevatorMotorConfig.p, ElevatorMotorConfig.i, ElevatorMotorConfig.d);

        twoConfig = new SparkMaxConfig();
        twoConfig
                .inverted(ElevatorMotorConfig.twoInverted)
                .idleMode(ElevatorMotorConfig.idleMode);
        twoConfig.encoder
                .positionConversionFactor(ElevatorMotorConfig.positionConversionFactor)
                .velocityConversionFactor(ElevatorMotorConfig.velocityConversionFactor);
        twoConfig.closedLoop
                .feedbackSensor(ElevatorMotorConfig.feedbackSensor)
                .pid(ElevatorMotorConfig.p, ElevatorMotorConfig.i, ElevatorMotorConfig.d);

        stageOne.configure(oneConfig, ElevatorMotorConfig.resetMode, ElevatorMotorConfig.persistMode);
        stageTwo.configure(twoConfig, ElevatorMotorConfig.resetMode, ElevatorMotorConfig.persistMode);
    }

    public Command move(DoubleSupplier input){
        return run(() -> {
            stageOne.set(input.getAsDouble());
            stageTwo.set(input.getAsDouble());
        });
    }
}
