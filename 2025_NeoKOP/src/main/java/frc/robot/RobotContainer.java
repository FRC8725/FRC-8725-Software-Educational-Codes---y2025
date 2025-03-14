package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.DriveMotorSubsystem;

public class RobotContainer {
	private final GamepadJoystick joystick = new GamepadJoystick(GamepadJoystick.CONTROLLER_PORT);
	private final DriveMotorSubsystem driveMotorSubsystem = new DriveMotorSubsystem();
	private final DriveCmd driveCmd = new DriveCmd(driveMotorSubsystem, joystick);

	public RobotContainer() {
		this.driveMotorSubsystem.setDefaultCommand(this.driveCmd);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
