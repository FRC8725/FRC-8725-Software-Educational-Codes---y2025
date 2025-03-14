package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        this.registerDashboard();
        this.frontLeft = new SwerveModule(
            2, 1, 9,
            true, true,
            "frontLeft");
        this.frontRight = new SwerveModule(
            6, 5, 10,
            false, true,
            "frontRight");
        this.backLeft = new SwerveModule(
            4, 3, 11,
            true, true,
            "backLeft");
        this.backRight = new SwerveModule(
            7, 8, 12,
            false, true,
            "backRight");
        this.gyro = new AHRS(NavXComType.kMXP_UART);
        this.odometry = new SwerveDriveOdometry(
            SwerveConstants.swerveDriveKinematics, this.gyro.getRotation2d(), this.getModulePosition()
        );
        this.gyro.reset();
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), getModulePosition());
    }

    public void resetGyro() {
        this.gyro.reset();
    }

    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] state = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(field ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d()) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        this.setModuleState(state);
    }

    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public SwerveModuleState[] getModuleState() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        this.odometry.resetPosition(this.gyro.getRotation2d(), this.getModulePosition(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swerveDriveKinematics.toChassisSpeeds(this.getModuleState());
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    @Override
    public void putDashboard() {
    }
}