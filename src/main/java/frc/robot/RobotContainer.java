// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CloseIntakeCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.TranslationDriveCommand;
import frc.robot.commands.WinchPositionCommand;
import frc.robot.commands.RotationDriveCommand;
import frc.robot.commands.IdleDriveCommand;
import frc.robot.commands.IdleElevatorCommand;
import frc.robot.commands.IdleWinchCommand;
import frc.robot.commands.OpenIntakeCommand;
import frc.robot.commands.ReleaseSuctionCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.ExtensionElevatorCommand;
import frc.robot.commands.RotationElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final WinchSubsystem m_winchSubsystem = new WinchSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private static double m_drivePowerCap = 0.75;
  private static double m_rotatePower = 0;

  private final UsbCamera m_subsystemCamera;

  private final Spark m_blinkin;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(m_driveController.getRawAxis(1), 0.05, m_drivePowerCap) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driveController.getRawAxis(0), 0.05, m_drivePowerCap) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> m_rotatePower * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(
        m_elevatorSubsystem,
        m_winchSubsystem, 
        () -> -deadband(m_operatorController.getRawAxis(5), 0.05), 
        () -> -deadband(m_operatorController.getRawAxis(1), 0.05)
    ));

    m_subsystemCamera = CameraServer.startAutomaticCapture(0);
    m_subsystemCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    m_blinkin = new Spark(Constants.BLINKIN);

    // Configure the button bindings
    configureButtonBindings();
  }

  public void reset(int mode) {
    m_elevatorSubsystem.resetEncoders();
    m_winchSubsystem.resetEncoders();
    if (mode == 1) {
      m_drivetrainSubsystem.zeroGyroscope();
      m_drivetrainSubsystem.updateDistance();
    }
  }

  public void cancelElevatorCommands() {
    if (m_elevatorSubsystem.getCurrentCommand() != null) {
      m_elevatorSubsystem.getCurrentCommand().cancel();
    }
    if (m_winchSubsystem.getCurrentCommand() != null) {
      m_winchSubsystem.getCurrentCommand().cancel();
    }
  }

  public SequentialCommandGroup autonomousCommands(int alliance, boolean chargeStation) {
      if (alliance == 2 && chargeStation) {
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8),
              new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6)
              
          ),
          new OpenIntakeCommand(m_intakeSubsystem),
          new WaitCommand(0.6),
          new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new IdleElevatorCommand(m_elevatorSubsystem, 300),
                  new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8)
                  
              ),
              new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          ),
          new TranslationDriveCommand(m_drivetrainSubsystem, -3.5, 0, 1.7),
          new IdleDriveCommand(m_drivetrainSubsystem, 400),
          new TranslationDriveCommand(m_drivetrainSubsystem, 0, -1.22, 1.7),
          new IdleDriveCommand(m_drivetrainSubsystem, 300),
          new RotationDriveCommand(m_drivetrainSubsystem, 90, Math.PI / 4),
          new IdleDriveCommand(m_drivetrainSubsystem, 300),
          new TranslationDriveCommand(m_drivetrainSubsystem, 2.15, 0, 0.8)
          
          
        );
      }
      
      if (alliance == 0 && chargeStation) {
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8),
              new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6)
              
          ),
          new OpenIntakeCommand(m_intakeSubsystem),
          new WaitCommand(0.6),
          new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new IdleElevatorCommand(m_elevatorSubsystem, 300),
                  new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8)
              ),
              new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          ),
          new TranslationDriveCommand(m_drivetrainSubsystem, -3.5, 0, 1.7),
          new IdleDriveCommand(m_drivetrainSubsystem, 400),
          new TranslationDriveCommand(m_drivetrainSubsystem, 0, 1.22, 1.7),
          new IdleDriveCommand(m_drivetrainSubsystem, 300),
          new RotationDriveCommand(m_drivetrainSubsystem, 90, Math.PI / 4),
          new IdleDriveCommand(m_drivetrainSubsystem, 300),
          new TranslationDriveCommand(m_drivetrainSubsystem, 2.15, 0, 0.8)
          
        );
      }
      
      if (alliance == 1 && chargeStation) {
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8),
              new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6)
              
          ),
          new OpenIntakeCommand(m_intakeSubsystem),
          new WaitCommand(0.6),
          new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new IdleElevatorCommand(m_elevatorSubsystem, 500),
                  new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8)
              ),
              new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          ),
          new RotationDriveCommand(m_drivetrainSubsystem, 90, Math.PI / 6),
          new IdleDriveCommand(m_drivetrainSubsystem, 300),
          new TranslationDriveCommand(m_drivetrainSubsystem, -2.4, 0, 0.8)
        );
      }

      if (alliance == 2 && !chargeStation) {
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8),
              new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6)
              
          ),
          new OpenIntakeCommand(m_intakeSubsystem),
          new WaitCommand(0.6),
          new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new IdleElevatorCommand(m_elevatorSubsystem, 500),
                  new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8)
              ),
              new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          ),
          new TranslationDriveCommand(m_drivetrainSubsystem, -3.1, 0, 1.7),
          new IdleDriveCommand(m_drivetrainSubsystem, 400),
          new RotationDriveCommand(m_drivetrainSubsystem, 180, Math.PI / 2)
          // new IdleDriveCommand(m_drivetrainSubsystem, 300),
          // new ParallelCommandGroup(
          //     new TranslationDriveCommand(m_drivetrainSubsystem, -0.6, 0, 1.7),
          //     new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8),
          //     new WinchPositionCommand(m_winchSubsystem, "IN", 0.8)
          // ),
          // new CloseIntakeCommand(m_intakeSubsystem),
          // new IdleDriveCommand(m_drivetrainSubsystem, 300),
          // new ParallelCommandGroup(
          //   new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8),
          //   new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          // ),
          // new RotationDriveCommand(m_drivetrainSubsystem, 180, Math.PI / 2)
          // ,
          // new TranslationDriveCommand(m_drivetrainSubsystem, 4.4, 0, 1.5),
          // new ParallelCommandGroup(
          //   new ElevatorPositionCommand(m_elevatorSubsystem, "MID", 0.8),
          //   new WinchPositionCommand(m_winchSubsystem, "OUT", 0.7)
          // ),
          // new ParallelCommandGroup(
          //     new OpenIntakeCommand(m_intakeSubsystem),
          //     new IdleWinchCommand(m_winchSubsystem, 300)
          // )
          
        );
      }
      
      if (alliance == 0 && !chargeStation) {
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8),
              new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6)
              
          ),
          new OpenIntakeCommand(m_intakeSubsystem),
          new WaitCommand(0.6),
          new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new IdleElevatorCommand(m_elevatorSubsystem, 500),
                  new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8)
              ),
              new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          ),
          new TranslationDriveCommand(m_drivetrainSubsystem, -3.1, 0, 1.7),
          new IdleDriveCommand(m_drivetrainSubsystem, 400),
          new RotationDriveCommand(m_drivetrainSubsystem, 180, Math.PI / 2)
          // new IdleDriveCommand(m_drivetrainSubsystem, 300),
          // new ParallelCommandGroup(
          //     new TranslationDriveCommand(m_drivetrainSubsystem, -0.6, 0, 1.7),
          //     new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8),
          //     new WinchPositionCommand(m_winchSubsystem, "IN", 0.8)
          // ),
          // new CloseIntakeCommand(m_intakeSubsystem),
          // new IdleDriveCommand(m_drivetrainSubsystem, 300),
          // new ParallelCommandGroup(
          //   new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8),
          //   new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          // ),
          // new RotationDriveCommand(m_drivetrainSubsystem, 180, Math.PI / 2)
          // ,
          // new TranslationDriveCommand(m_drivetrainSubsystem, 4.4, 0, 1.5),
          // new ParallelCommandGroup(
          //   new ElevatorPositionCommand(m_elevatorSubsystem, "MID", 0.8),
          //   new WinchPositionCommand(m_winchSubsystem, "OUT", 0.7)
          // ),
          // new ParallelCommandGroup(
          //     new OpenIntakeCommand(m_intakeSubsystem),
          //     new IdleWinchCommand(m_winchSubsystem, 300)
          // )
          
        );
      }
      
      if (alliance == 1  && !chargeStation) {
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8),
              new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6)
              
          ),
          new OpenIntakeCommand(m_intakeSubsystem),
          new WaitCommand(0.6),
          new WinchPositionCommand(m_winchSubsystem, "A-OUT", 0.6),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new IdleElevatorCommand(m_elevatorSubsystem, 500),
                  new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8)
              ),
              new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)
          )
        );
      }
    return new SequentialCommandGroup(new IdleDriveCommand(m_drivetrainSubsystem));
  }

  public SequentialCommandGroup testAutonSpeed(double speed) {
    return new SequentialCommandGroup(
        new RotationDriveCommand(m_drivetrainSubsystem, 90, speed),
        new IdleDriveCommand(m_drivetrainSubsystem, 1500),
        new RotationDriveCommand(m_drivetrainSubsystem, -180, speed),
        new IdleDriveCommand(m_drivetrainSubsystem)
    );
  
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link  
   * edu.wpi.first.wpilibj.Joystick}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Operator start button cancels presets
    Button m_cancelPresets = new Button(() -> m_operatorController.getRawButton(8));
    m_cancelPresets.whenPressed(() -> cancelElevatorCommands());
    
    // Driver top-right thumbpad button zeroes gyroscope
    Button m_resetGyro = new Button(() -> m_driveController.getRawButton(6));
    m_resetGyro.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // Operator back and start buttons zeroes subsystems
    Button m_resetElevator = new Button(() -> m_operatorController.getRawButton(7) && m_operatorController.getRawButton(8));
    m_resetElevator.whenPressed(() -> reset(0));

    // Operator left bumper releases intake suction
    Button m_releaseSuction = new Button(() -> m_operatorController.getRawButton(5));
    m_releaseSuction.whenPressed(new ReleaseSuctionCommand(m_intakeSubsystem));

    // Driver holding down driver trigger activates turbo speed
    Button m_turbo = new Button(() -> m_driveController.getRawButton(1));
    m_turbo.whenPressed(() -> setTurbo(1.0));
    m_turbo.whenReleased(() -> setTurbo(0.5));

    // Driver holding down side thumb button activates hard brakes
    Button m_brake = new Button(() -> m_driveController.getRawButton(10));
    m_brake.whileHeld(() -> setIdleMode(0));
    m_brake.whenReleased(() -> setIdleMode(1));

    // Operator 'A' button sets elevator to low position
    Button m_lowPosition = new Button(() -> m_operatorController.getRawButton(1));
    m_lowPosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8), new WinchPositionCommand(m_winchSubsystem, "IN", 1)));

    // Operator 'X' button sets elevator to mid position
    Button m_midPosition = new Button(() -> m_operatorController.getRawButton(3));
    m_midPosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "MID", 0.8), new WinchPositionCommand(m_winchSubsystem, "OUT", 0.8)));

    // Operator 'Y' button sets elevator to high position
    Button m_highPosition = new Button(() -> m_operatorController.getRawButton(4));
    m_highPosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "HIGH", 0.8), new WinchPositionCommand(m_winchSubsystem, "OUT", 0.8)));

    // Operator 'B' button sets winch to drive position
    Button m_drivePosition = new Button(() -> m_operatorController.getRawButton(2));
    m_drivePosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "LOW", 0.8), new WinchPositionCommand(m_winchSubsystem, "DRIVE", 0.8)));

    // Operator left bumper sets elevator to human player intake position
    Button m_humanPosition = new Button(() -> m_operatorController.getRawButton(6));
    m_humanPosition.whenPressed(new ParallelCommandGroup(new ElevatorPositionCommand(m_elevatorSubsystem, "HUMAN", 0.8), new WinchPositionCommand(m_winchSubsystem, "HUMAN", 0.8)));

    // Operator left trigger opens claw
    Button m_openClaw = new Button(() -> m_operatorController.getRawAxis(2) > 0.5);
    m_openClaw.whenPressed(new OpenIntakeCommand(m_intakeSubsystem));

    // Operator right trigger closes claw
    Button m_closeClaw = new Button(() -> m_operatorController.getRawAxis(3) > 0.5);
    m_closeClaw.whenPressed(new CloseIntakeCommand(m_intakeSubsystem));

    // Driver button 11 rotates left
    Button m_rotateLeft = new Button(() -> m_driveController.getRawButton(11));
    m_rotateLeft.whileHeld(() -> setRotatePower("left"));
    m_rotateLeft.whenReleased(() -> setRotatePower("none"));

    // Driver button 12 rotates robot right
    Button m_rotateRight = new Button(() -> m_driveController.getRawButton(12));
    m_rotateRight.whileHeld(() -> setRotatePower("right"));
    m_rotateRight.whenReleased(() -> setRotatePower("none"));

    Button m_yellow = new Button(() -> m_driveController.getRawButton(7));
    m_yellow.whenPressed(() -> m_blinkin.set(0.69));

    Button m_purple = new Button(() -> m_driveController.getRawButton(8));
    m_purple.whenPressed(() -> m_blinkin.set(0.37));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  } 

  private static double modifyAxis(double value, double deadband, double limit) {
    // Deadband
    value = deadband(value, deadband);

    // Cube the axis and set the limit
    value = value * limit;
    return value;
    
  }

  private static void setTurbo(double power) 
  {
    m_drivePowerCap = power;
  }

  public void setIdleMode(int mode) {
    m_drivetrainSubsystem.setIdleMode(mode);
  }

  public void setRotatePower(String state) {
    if (state.equals("left")) {
      m_rotatePower = 0.15;
    }
    else if (state.equals("right")) {
      m_rotatePower = -0.15;
    }
    else {
      m_rotatePower = 0;
    }
  }
}