package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsytems.Spin;
import frc.robot.subsytems.Swerve;
import frc.thunder.LightningContainer;

public class RobotContainer extends LightningContainer{
  
	Swerve drivetrain = TunerConstants.DriveTrain; 
	Spin spin = new Spin();
	
	XboxController driver = new XboxController(0);
  
	SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true); // I want field-centric driving in open loop
  	SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  	Telemetry logger = new Telemetry(Constants.MaxSpeed);

	// NamedCommands.registerCommand("spin20" ? () -> new SpinControl(spin, (() -> 20)));

  	Command runAuto = drivetrain.getAutoPath("teehee");

	@Override
	protected void configureButtonBindings() {
		// new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(() -> brake));
		// new Trigger(driver::getBButton).whileTrue(drivetrain
    //     	.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
		// new Trigger(driver::getXButton).onTrue(drivetrain::tareEverything); //TODO What does this do?

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90))); //This is where they had it
    // }
    
  }

  	@Override
	protected void configureDefaultCommands() {
		drivetrain.setDefaultCommand(drivetrain.applyRequest(
			() -> drive.withVelocityX(-MathUtil.applyDeadband(driver.getLeftY(), 0.1) * Constants.MaxSpeed) // Drive forward with negative Y (forward)
			.withVelocityY(-MathUtil.applyDeadband(driver.getLeftX(), 0.1) * Constants.MaxSpeed) // Drive left with negative X (left)
			.withRotationalRate(-MathUtil.applyDeadband(driver.getRightX(), 0.1) * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
		));
		drivetrain.registerTelemetry(logger::telemeterize); // TODO Does this go here?
	}

	@Override
	protected void configureAutonomousCommands() {}

	@Override
	protected void releaseDefaultCommands() {}
	
	@Override
	protected void initializeDashboardCommands() {}
	
	@Override
	protected void configureFaultCodes() {}
	
	@Override
	protected void configureFaultMonitors() {}

	@Override
	protected void configureSystemTests() {}

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return runAuto;
  }
}
