package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorReverse;
import frc.robot.DeviceId.DriveMotor;

public class DriveMotorSubsystem extends SubsystemBase {
    private final DriveMotorModule leftModule;
    private final DriveMotorModule rightModule;

    public DriveMotorSubsystem() {
        this.leftModule = new DriveMotorModule(DriveMotor.FRONT_LEFT, DriveMotor.BACK_LEFT,
            MotorReverse.FRONT_LEFT, MotorReverse.BACK_LEFT
        );
        this.rightModule = new DriveMotorModule(DriveMotor.FRONT_RIGHT, DriveMotor.BACK_RIGHT,
            MotorReverse.FRONT_RIGHT, MotorReverse.BACK_RIGHT
        );
    }
    
    public void move(double leftSpeed, double rightSpeed) {
        this.leftModule.setDesiredState(leftSpeed);
        this.rightModule.setDesiredState(rightSpeed);
    }

    public void stopModules() {
        this.leftModule.stop();
        this.rightModule.stop();
    }
}
