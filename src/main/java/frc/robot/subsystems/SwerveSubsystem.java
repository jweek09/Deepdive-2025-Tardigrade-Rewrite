package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveSubsystem implements Subsystem {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    public static SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(15);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        this.setupPathPlanner();
    }

    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose,
                    // Robot pose supplier
                    this::resetOdometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces()
                            );
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)
                            // Rotation PID constants
                    ),
                    config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
                    // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            DriverStation.reportError("Failed to initialize swerveSubsystem PathPlanner: " + e.getMessage(), e.getStackTrace());
            // Consider whether the robot should continue or disable itself
        } }
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }
    public SwerveIMU fetchIMU() {
        return swerveDrive.getGyro();
    }
    public static SwerveDrive fetchSwerve() {
        return swerveDrive;
    }
    public void driveFieldOriented(ChassisSpeeds velocity ) {
        swerveDrive.driveFieldOriented(velocity);
    }
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run( () -> swerveDrive.driveFieldOriented(velocity.get()));
    }
    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                                DoubleSupplier headingY)
    {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }
    public Command centerModulesCommand()
    {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }
    public Command resetOdometryInFrontOfBlueReef() {
        Pose2d inFrontOfBlueReef = new Pose2d(7, 4, Rotation2d.fromDegrees(180));
        return run( () -> {
            this.resetOdometry(inFrontOfBlueReef);
        });
    }

}
