package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;

import java.util.function.DoubleSupplier;

public class RotationDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_arcLength;
    private final DoubleSupplier m_rotationSupplier;

    public RotationDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               double angle, 
                               double avelocity) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_arcLength = (Math.abs(angle) / 360.0) * Math.PI * Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS, DRIVETRAIN_WHEELBASE_METERS);
        this.m_rotationSupplier = () -> Math.copySign(1, angle) * avelocity;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    m_rotationSupplier.getAsDouble(),
                    m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );
    }

    @Override
    public boolean isFinished() {
        if (m_drivetrainSubsystem.getDistanceTravelled() < m_arcLength) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_drivetrainSubsystem.updateDistance();
    }
}
