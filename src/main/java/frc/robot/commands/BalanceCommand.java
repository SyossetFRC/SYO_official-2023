package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final PIDController m_pid;

    public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        this.m_pid = new PIDController(0.005, 0, 0);
        this.m_pid.setTolerance(3);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                        0,
                        -m_pid.calculate(m_drivetrainSubsystem.getRoll(), 0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        0
                )
        );
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
