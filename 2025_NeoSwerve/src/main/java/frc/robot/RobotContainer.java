package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private final Driver driver = new Driver();
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	
	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(new SwerveDriveCmd(
			this.swerveSubsystem,
			this.driver::getXDesiredSpeed, this.driver::getYDesiredSpeed, this.driver::getRotationSpeed));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}