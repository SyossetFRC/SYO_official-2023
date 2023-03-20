package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class TranslationDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_distance;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

    public TranslationDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               double distanceX,
                               double distanceY, 
                               double velocity) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_distance = Math.hypot(distanceX, distanceY);
        this.m_translationXSupplier = () -> (distanceX / this.m_distance) * velocity;
        this.m_translationYSupplier = () -> (distanceY / this.m_distance) * velocity;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    0,
                    m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );
    }

    @Override
    public boolean isFinished() {
        if (m_drivetrainSubsystem.getDistanceTravelled() < m_distance) {
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
