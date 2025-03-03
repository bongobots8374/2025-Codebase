package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrawbridgeMotorConfig;
import frc.robot.Constants.CanIDs;

import java.util.function.DoubleSupplier;

public class Drawbridge extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig config;

    public Drawbridge() {
        motor = new SparkMax(CanIDs.drawbridgeMotor, SparkLowLevel.MotorType.kBrushless);

        config = new SparkMaxConfig();
        config
                .inverted(DrawbridgeMotorConfig.inverted)
                .idleMode(DrawbridgeMotorConfig.idleMode);
        config.encoder
                .positionConversionFactor(DrawbridgeMotorConfig.positionConversionFactor)
                .velocityConversionFactor(DrawbridgeMotorConfig.velocityConversionFactor);
        config.closedLoop
                .feedbackSensor(DrawbridgeMotorConfig.feedbackSensor)
                .pid(DrawbridgeMotorConfig.p, DrawbridgeMotorConfig.i, DrawbridgeMotorConfig.d);

        motor.configure(config, DrawbridgeMotorConfig.resetMode, DrawbridgeMotorConfig.persistMode);
    }

    public Command move(DoubleSupplier speed) {
        return run(() -> {
            motor.set(speed.getAsDouble());
        });
    }
}
