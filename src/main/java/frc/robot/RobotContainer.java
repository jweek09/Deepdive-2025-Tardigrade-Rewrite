// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem kSwerveSubsystem = new SwerveSubsystem();


    // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController kDriverController =
      new CommandXboxController(SwerveConstants.kDriverControllerPort);
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomous")
            .add("Autonomous Mode", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
    Shuffleboard.getTab("Reset Odometry:")
            .add("Reset in front of blue reef", kSwerveSubsystem.resetOdometryInFrontOfBlueReef())
            .withWidget(BuiltInWidgets.kCommand);

    // Configure the trigger bindings
    configureBindings();

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(kSwerveSubsystem.fetchSwerve(),
                    () -> kDriverController.getLeftY() * -1,
                    () -> kDriverController.getLeftX() * -1)
            .withControllerRotationAxis(kDriverController::getRightX)
            .deadband(Constants.SwerveConstants.kDeadband)
            .scaleTranslation(1)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis
            (kDriverController::getRightX, kDriverController::getRightY).headingWhile(true);

    kSwerveSubsystem.fetchIMU().factoryDefault();
    kSwerveSubsystem.fetchIMU().clearStickyFaults();
    kSwerveSubsystem.fetchSwerve().setAutoCenteringModules(SwerveConstants.kWheelsAutoCenter);
    kSwerveSubsystem.fetchSwerve().setHeadingCorrection(SwerveConstants.kYAGSLHeadingCorrection);

    Command driveFieldOrientedDirectAngle = kSwerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = kSwerveSubsystem.driveFieldOriented(driveAngularVelocity);

    kSwerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

//    Command driveFieldOrientedDirectAngle = kSwerveSubsystem.driveCommand(
//            () -> MathUtil.applyDeadband(kDriverController.getLeftY(), .5),
//            () -> MathUtil.applyDeadband(kDriverController.getLeftX(), .5),
//            () -> kDriverController.getRightX(),
//            () -> kDriverController.getRightY());
//
//    kSwerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
  ;}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    kDriverController.b().onTrue(kSwerveSubsystem.centerModulesCommand());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }}
