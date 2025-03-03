package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberMotorConfig;
import frc.robot.Constants.CanIDs;

import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig config;
    
    public Climber() {
        motor = new SparkMax(CanIDs.climberMotor, SparkLowLevel.MotorType.kBrushless);
        
        config = new SparkMaxConfig();
        config
                .inverted(ClimberMotorConfig.inverted)
                .idleMode(ClimberMotorConfig.idleMode);
        config.encoder
                .positionConversionFactor(ClimberMotorConfig.positionConversionFactor)
                .velocityConversionFactor(ClimberMotorConfig.velocityConversionFactor);
        config.closedLoop
                .feedbackSensor(ClimberMotorConfig.feedbackSensor)
                .pid(ClimberMotorConfig.p, ClimberMotorConfig.i, ClimberMotorConfig.d);

        motor.configure(config, ClimberMotorConfig.resetMode, ClimberMotorConfig.persistMode);
    }

    public Command move(DoubleSupplier speed) {
        return run(() -> {
            motor.set(speed.getAsDouble());
        });
    }
}
