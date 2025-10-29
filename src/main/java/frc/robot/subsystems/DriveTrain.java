package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;


/**
 * the drivetrain subsystem.
 * 
 * 
 */
public class DriveTrain implements Subsystem{
    // the subsystem wide variables
    PIDController drivePID;
    PIDController rotPID;
    
    SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        DTConstants.frontLeftLocation,
        DTConstants.frontRightLocation,
        DTConstants.backLeftLocation,
        DTConstants.backRightLocation
    );
        
    SwerveModule[] swerveModules = new SwerveModule[] { //fl,fr,bl,br
            new SwerveModule(DTConstants.frontLeftDriveID, DTConstants.frontLeftRotationID), 
            new SwerveModule(DTConstants.frontRightDriveID,DTConstants.frontRightRotationID),
            new SwerveModule(DTConstants.backLeftDriveID,DTConstants.backLeftRotationID), 
            new SwerveModule(DTConstants.backRightDriveID, DTConstants.backRightRotationID),
    };
               
    public DriveTrain(){
        drivePID = new PIDController(DTConstants.drkP,DTConstants.drkI,DTConstants.drkD);
        rotPID = new PIDController(DTConstants.rotkP,DTConstants.rotkI,DTConstants.rotkD);   
    }

    public void driveRobotRelative(PS5Controller driveController){
        //chassis speeds based on drive controller
        ChassisSpeeds speed = new ChassisSpeeds(
            Math.pow(-driveController.getLeftY(),3)*DTConstants.maxSpeed,
            Math.pow(driveController.getLeftX(),3)*DTConstants.maxSpeed,
            Math.pow(-driveController.getRightX(),3)*DTConstants.maxRotation);
        driveFromChassisSpeeds(speed);
    }
    /**
     * good luck
     * @param speed the chassis speeds object you get from another drive funtion
     */
    public void driveFromChassisSpeeds(ChassisSpeeds speed){

        //the module states from the chassis speed
        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(speed); //99% sure this is supposed to be within a function
        
        //I'm not sorry.
        for(int i = 0; i < 4;i++){
            swerveModules[i].driveWithVoltage(
                drivePID.calculate(
                    swerveModules[i].driveMotor.getBusVoltage(), //yes, the measurement and setpoint are different units.  
                    moduleStates[i].speedMetersPerSecond), //we're pretty sure it works. we hope it works.
                rotPID.calculate(
                    swerveModules[i].turnMotor.getAbsoluteEncoder().getPosition(),
                    moduleStates[i].angle.getRadians()));
        } 
    }
}