package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
public class DriveTrain implements Subsystem{
    
    SwerveModule frontLeft;
    SwerveModule frontRight;      
    SwerveModule backLeft;
    SwerveModule backRight;
    ChassisSpeeds speeds = new ChassisSpeeds(3.0, -2.0, Math.PI); // not supposed to be hardcoded i think

    SwerveDriveKinematics kine = new SwerveDriveKinematics(
        DTConstants.frontLeftLocation,
        DTConstants.frontRightLocation,
        DTConstants.backLeftLocation,
        DTConstants.backRightLocation
        );
    SwerveModuleState[] moduleStates = kine.toSwerveModuleStates(speeds);
    SwerveModuleState frontLeftState = moduleStates[0];
    SwerveModuleState frontRightState = moduleStates[1];
    SwerveModuleState backLefState = moduleStates[2];
    SwerveModuleState backRightState = moduleStates[3];
    
               
    public DriveTrain(){
        
        frontRight = new SwerveModule(-1,-1); //TODO: find the IDs???????
        frontLeft = new SwerveModule(-1,-1); //TODO: find the IDs???????
        backRight = new SwerveModule(-1,-1); //TODO: find the IDs???????
        backLeft = new SwerveModule(-1,-1); //TODO: find the IDs???????

    }
    public void driveOrSmthIGuessIdk(PS5Controller driveController){
        ChassisSpeeds speed = new ChassisSpeeds(
            Math.pow(-driveController.getLeftY(),3)*DTConstants.maxSpeed,
            Math.pow(driveController.getLeftX(),3)*DTConstants.maxSpeed,
            Math.pow(-driveController.getRightX(),3)*DTConstants.maxRotation);       
    }
}
