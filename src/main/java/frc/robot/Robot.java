// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  RobotContainer m_container;

  SendableChooser<Integer> positionChooser = new SendableChooser<>();
  SendableChooser<Boolean> chargeChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_container = new RobotContainer();

    positionChooser.addOption("Left Auton", 0);
    positionChooser.addOption("Mid Auton", 1);
    positionChooser.setDefaultOption("Right Auton", 2);

    SmartDashboard.putData(positionChooser);

    chargeChooser.addOption("Yes charge", true);
    chargeChooser.setDefaultOption("No charge", false);

    SmartDashboard.putData(chargeChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Auton Pos", positionChooser.getSelected());
    SmartDashboard.putBoolean("Charger?", chargeChooser.getSelected());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    m_container.reset(1);
    m_container.setIdleMode(0);

    m_container.autonomousCommands(positionChooser.getSelected(), chargeChooser.getSelected()).schedule();

    //AUTON TEST ONLY DO NOT UNCOMMENT
    //m_container.testAuton().schedule();;
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
