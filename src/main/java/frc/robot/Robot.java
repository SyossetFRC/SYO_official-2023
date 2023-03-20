// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  RobotContainer m_container;
  String m_autonPosition;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_container = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    boolean autonPositionLeft = Shuffleboard.getTab("Drivetrain")
        .add("Left Auton", false)
        .withWidget("Toggle Button")
        .getEntry()
        .getBoolean(false);

    boolean autonPositionMid = Shuffleboard.getTab("Drivetrain")
        .add("Mid Auton", false)
        .withWidget("Toggle Button")
        .getEntry()
        .getBoolean(false);

    boolean autonPositionRight = Shuffleboard.getTab("Drivetrain")
        .add("Right Auton", false)
        .withWidget("Toggle Button")
        .getEntry()
        .getBoolean(false);

    if (autonPositionLeft) {
      m_autonPosition = "left";
    }
    if (autonPositionMid) {
      m_autonPosition = "mid";
    }
    if (autonPositionRight) {
      m_autonPosition = "right";
    }

    boolean resetSubsystem = Shuffleboard.getTab("LiveWindow")
        .add("Reset Subsystems", false)
        .withWidget("Toggle Button")
        .getEntry()
        .getBoolean(false);

    if (resetSubsystem) {
      m_container.reset(0);
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    m_container.reset(1);
    m_container.setIdleMode(0);

    m_container.autonomousCommands(m_autonPosition).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CommandScheduler.getInstance().cancelAll();
    
    m_container.setIdleMode(1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}
}
