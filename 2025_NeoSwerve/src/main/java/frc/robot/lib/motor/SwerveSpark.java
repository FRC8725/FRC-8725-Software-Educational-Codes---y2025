package frc.robot.lib.motor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.SwerveConstants;

public class SwerveSpark extends SparkMax {
    private final double GEAR_RATIO;

    public SwerveSpark(int motorPort, boolean reverse, double gearRatio) {
        super(motorPort, MotorType.kBrushless);
        this.GEAR_RATIO = gearRatio;
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .inverted(reverse)
            .smartCurrentLimit(SwerveConstants.CURRENT_LIMIT);

        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getMotorVelocity() {
        return super.getEncoder().getVelocity() * this.GEAR_RATIO * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI;
    }

    public double getMotorPosition() {
        return super.getEncoder().getPosition() * this.GEAR_RATIO * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI;
    }
}
