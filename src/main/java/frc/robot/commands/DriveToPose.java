package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class DriveToPose extends Command {
    private final Vision vision;
    private final SwerveSubsystem swerveSubsystem;

    ChassisSpeeds chassisSpeeds;

    public DriveToPose(Vision vision, SwerveSubsystem swerveSubsystem){
        this.vision = vision;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute(){
        double noteDistance = vision.getNoteDistance();

        double distanceError = 0.3 - noteDistance * -1;
        if (distanceError < -1 || distanceError > 1) {
            if (distanceError < -1) {distanceError = -1;}
            else {distanceError = 1;}
        }

        double driveAdjust = distanceError * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;



        chassisSpeeds = new ChassisSpeeds(driveAdjust, driveAdjust, 0.0);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }
}