// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private UsbCamera camera = CameraServer.startAutomaticCapture(0);
  private ShuffleboardTab tab_competition = Shuffleboard.getTab("Competition");

  // private IntakeSubsystem m_intake;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    tab_competition.add(camera).withSize(6, 4);
    // Set the position of the robot to be inverted so that the robot drive
    // backwards
    m_robotContainer.headingSet(180);
    // m_intake.getArmParamaters();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.headingSet(180);

    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // Limelight debug
  GenericEntry rangeOnboardEntry = Shuffleboard.getTab("Competition")
      .add("Target X Offset", 0)
      .getEntry();
  GenericEntry rangeOnboardEntry2 = Shuffleboard.getTab("Competition")
      .add("Target Found", false)
      .getEntry();
  GenericEntry rangeOnboardEntry3 = Shuffleboard.getTab("Competition")
      .add("Arm Angle", 0)
      .getEntry();
  GenericEntry robotYaw = Shuffleboard.getTab("Competition")
      .add("Yaw", 0)
      .getEntry();

  // Competition tab
  GenericEntry targetFound = Shuffleboard.getTab("Competition").add("Ready To Align", false).getEntry();
  GenericEntry robotAligned = Shuffleboard.getTab("Competition").add("Robot Aligned", false).getEntry();
  GenericEntry robotRing = Shuffleboard.getTab("Competition").add("Robot Has Ring", false).getEntry();
  GenericEntry distance = Shuffleboard.getTab("Competition").add("Distance", 0).getEntry();
  GenericEntry tagId = Shuffleboard.getTab("Competition").add("Tag ID", 0).getEntry();

  // Toggle button

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    rangeOnboardEntry.setDouble(m_robotContainer.getLimelightX());
    rangeOnboardEntry2.setBoolean(m_robotContainer.targetValid());
    targetFound.setBoolean(m_robotContainer.targetValid());
    robotAligned.setBoolean(m_robotContainer.isRobotAligned());
    rangeOnboardEntry3.setDouble(m_robotContainer.getArmAngle());
    robotRing.setBoolean(m_robotContainer.hasRingR());
    robotYaw.setDouble(m_robotContainer.getYawR());
    distance.setDouble(m_robotContainer.getDistance());
    tagId.setDouble(m_robotContainer.getAprilTagID());

    // Normalize the robot heading
    // This will prevent the robot from reading over 360 degrees
    m_robotContainer.normalizeAngle(m_robotContainer.getYawR());
    m_robotContainer.updateDrive();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
