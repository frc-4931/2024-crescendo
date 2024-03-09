package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class LockOnNote extends Command{
     private final Vision vision;
    private final SwerveSubsystem swerveSubsystem;

    ChassisSpeeds chassisSpeeds;

    public LockOnNote(Vision vision, SwerveSubsystem swerveSubsystem){
        this.vision = vision;
        this.swerveSubsystem = swerveSubsystem;
    }


 @Override
    public void execute(){        
        if(!(vision.valid())) {return;}
        while(vision.valid()) {
            //if(vision.getTx() < -1) {
            double pid = swerveSubsystem.getHeading() - vision.getTx() *-1;

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(pid, pid, DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);
            }
            //else {
            //if(vision.getTx() > 1) {

        //     }
        // }
   // }
}}
