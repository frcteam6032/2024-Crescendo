// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ComputerAlign;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakePickupArmDown;
import frc.robot.commands.IntakePickupArmUp;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.Shoot;
import frc.robot.commands.Wench;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WenchSubsystem;

import java.util.List;
import frc.robot.commands.AutomaticIntake; // Import the missing class

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final VisionSubsystem m_limelight = new VisionSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeWheels m_wheels = new IntakeWheels();
  //private final WenchSubsystem m_whench = new WenchSubsystem();


  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private final Command ComputerAligner = new ComputerAlign(m_robotDrive, m_limelight);
  private final Command PickupUp = new IntakePickupArmUp(m_intake);
  private final Command PickupDown = new IntakePickupArmDown(m_intake);
  private final Command Shoot = new Shoot(m_shooter, m_wheels);
  private final Command AmpScore = new ScoreAmp(m_wheels, m_intake);
  private final Command IntakeWeelsIn = new IntakeIn(m_wheels, m_intake);
  private final Command IntakeWeelsOut = new IntakeOut(m_wheels);
  private final Command AutomaticIntake = new AutomaticIntake(m_wheels, m_intake);
  //private final Command WenchCmd = new Wench(m_whench);


SendableChooser < Command > m_chooser = new SendableChooser < > ();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("No Auto", null);
        // Top one already done because there is no auto routine
        m_chooser.addOption("Leave Auto", null);
        m_chooser.addOption("Score AMP", null);
        m_chooser.addOption("Score Speaker", null);


        // Put the chooser on the dashboard
        Shuffleboard.getTab("Competition")
            .add("Auto Chooser", m_chooser)
            .withPosition(6, 3)
            .withSize(2, 1);


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Setting up driver commands
    new Trigger(m_driverController::getYButton).whileTrue(ComputerAligner);
    new Trigger(m_driverController::getAButton).onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    // Setting up operator controls
    new Trigger(m_operatorController::getRightBumper).whileTrue(PickupUp);
    new Trigger(m_operatorController::getLeftBumper).whileTrue(PickupDown);
    new Trigger(m_operatorController::getBButton).whileTrue(Shoot);
    new Trigger(m_operatorController::getYButton).whileTrue(AmpScore);
    new Trigger(m_operatorController::getAButton).whileTrue(IntakeWeelsIn);
    new Trigger(m_operatorController::getXButton).whileTrue(IntakeWeelsOut);
    
    // Adds automatic intake
    new Trigger(m_operatorController::getStartButton).whileTrue(AutomaticIntake);
    
    // Adds wench
    //new Trigger(m_operatorController::getBackButton).whileTrue(WenchCmd);

}


// Setting up methods for the robot.java class to use to display information to the shuffleboard
public double getLimelightX() {
    return m_limelight.getTX();
}

public double getLimelightY() {
    return m_limelight.getTY();
}

public boolean isRobotAligned() {
    return m_limelight.isAligned();
}

public boolean targetValid() {
    return m_limelight.isTargetValid();
}

public boolean hasRingR() {
    return m_intake.hasRing();
}

public double getYawR() {
    return m_robotDrive.getHeading();
}


public double getArmAngle() {
    return m_intake.getAngle();  
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    /*
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)); */
     }
  }
  

