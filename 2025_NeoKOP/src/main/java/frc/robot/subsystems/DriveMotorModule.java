package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motors.KopSpark;

public class DriveMotorModule implements IDashboardProvider {
    private final KopSpark frontSpark;
    private final KopSpark backSpark;

    public DriveMotorModule(
        int frontId, int backId,
        boolean frontReverse, boolean backReverse
    ) {
        this.registerDashboard();
        this.frontSpark = new KopSpark(frontId, frontReverse);
        this.backSpark = new KopSpark(backId, backReverse);
    }

    public void setDesiredState(double speed) {
        this.frontSpark.set(speed);
        this.backSpark.set(speed);
    }

    public void stop() {
        this.frontSpark.stopMotor();
        this.backSpark.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Front Speed", this.frontSpark.getEncoder().getVelocity());
        SmartDashboard.putNumber("Back Speed", this.backSpark.getEncoder().getVelocity());
    }
}
