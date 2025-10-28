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
    SwerveModule frontLeft;
    SwerveModule frontRight;      
    SwerveModule backLeft;
    SwerveModule backRight;
    PIDController drivePID;
    PIDController rotPID;
    
    SwerveDriveKinematics kine = new SwerveDriveKinematics(
        DTConstants.frontLeftLocation,
        DTConstants.frontRightLocation,
        DTConstants.backLeftLocation,
        DTConstants.backRightLocation
        );

    
               
    public DriveTrain(){
        drivePID = new PIDController(DTConstants.drkP,DTConstants.drkI,DTConstants.drkD);
        rotPID = new PIDController(DTConstants.rotkP,DTConstants.rotkI,DTConstants.rotkD);
        
        frontRight = new SwerveModule(-1,-1); //TODO: find the IDs???????
        frontLeft = new SwerveModule(-1,-1); //TODO: find the IDs???????
        backRight = new SwerveModule(-1,-1); //TODO: find the IDs???????
        backLeft = new SwerveModule(-1,-1); //TODO: find the IDs???????

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
        SwerveModuleState[] moduleStates = kine.toSwerveModuleStates(speed); //99% sure this is supposed to be within a function
        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];
        
        //I'm so sorry.
        //Me Too.
        frontLeft.setVoltage(
            drivePID.calculate(
                frontLeft.driveMotor.getBusVoltage(), //yes, the measurement and setpoint are different units.  
                frontLeftState.speedMetersPerSecond),//we're pretty sure it works. we hope it works.
            rotPID.calculate(
                frontLeft.turnMotor.getAbsoluteEncoder().getPosition(),
                frontLeftState.angle.getRadians()));
        frontRight.setVoltage(
            drivePID.calculate(
                frontRight.driveMotor.getBusVoltage(),
                frontRightState.speedMetersPerSecond),
            rotPID.calculate(
                frontRight.turnMotor.getAbsoluteEncoder().getPosition(),
                frontRightState.angle.getRadians()));
        backLeft.setVoltage(
            drivePID.calculate(
                backLeft.driveMotor.getBusVoltage(),
                backLeftState.speedMetersPerSecond),
            rotPID.calculate(
                backLeft.turnMotor.getAbsoluteEncoder().getPosition(),
                backLeftState.angle.getRadians()));
        backRight.setVoltage(
            drivePID.calculate(
                backRight.driveMotor.getBusVoltage(),
                backRightState.speedMetersPerSecond),
            rotPID.calculate(
                backRight.turnMotor.getAbsoluteEncoder().getPosition(),
                backRightState.angle.getRadians()));   
    }
}
//lowk lock in
//lowk throw away the key