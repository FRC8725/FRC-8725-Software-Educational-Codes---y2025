package frc.robot.lib.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class KopSpark extends SparkMax {
    public KopSpark(int id, boolean reverse) {
        super(id, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kCoast)
            .inverted(reverse)
            .smartCurrentLimit(30);

        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
