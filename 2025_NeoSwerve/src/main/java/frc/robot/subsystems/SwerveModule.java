package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motor.SwerveSpark;
import frc.robot.lib.swerve.TurnEncoder;

public class SwerveModule implements IDashboardProvider {
    private final SwerveSpark driveMotor;
    private final SwerveSpark turnMotor;

    private final TurnEncoder turnEncoder;

    private final PIDController turnPid;

    private final String motorName;

    public SwerveModule(
        int driveMotorPort, int turnMotorPort, int turnEncoderPort,
        boolean driveMotorReverse, boolean turnMotorReverse,
        String motorName
    ){
        this.registerDashboard();

        this.driveMotor = new SwerveSpark(driveMotorPort, driveMotorReverse, SwerveConstants.DRIVE_GEAR_RATIO);
        this.turnMotor = new SwerveSpark(turnMotorPort, turnMotorReverse, SwerveConstants.TURN_GEAR_RATIO);

        this.turnEncoder = new TurnEncoder(turnEncoderPort);

        this.turnPid = new PIDController(2.35, 0.0, 0.0);
        this.turnPid.enableContinuousInput(-0.5, 0.5);

        this.motorName = motorName;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveMotor.getMotorVelocity(),
            Rotation2d.fromRotations(this.turnEncoder.getAbsolutePositionRotations()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveMotor.getMotorPosition(),
            Rotation2d.fromRotations(this.turnEncoder.getAbsolutePositionRotations()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        desiredState.optimize(this.getState().angle);

        double driveOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
        double turnOutput = this.turnPid.calculate(
                this.getState().angle.getRotations(), desiredState.angle.getRotations());

        this.driveMotor.set(driveOutput);
        this.turnMotor.set(turnOutput);
    }

    public void stop() {
        this.driveMotor.set(0.0);
        this.turnMotor.set(0.0);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.motorName + " Drive Pos", this.driveMotor.getMotorPosition());
        SmartDashboard.putNumber(this.motorName + " Drive Vel", this.driveMotor.getMotorVelocity());
        SmartDashboard.putNumber(this.motorName + " Turn Pose", this.turnEncoder.getAbsolutePositionRotations());
    }
}
