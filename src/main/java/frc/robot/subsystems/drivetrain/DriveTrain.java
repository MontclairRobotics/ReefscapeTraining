package frc.robot.subsystems.drivetrain;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// if it has title case, its james, if it has no caps it's or (yes that's his name) and dashiell is also here in case anyone forgot
/**
 * the drivetrain subsystem.
 * 
 */
public class DriveTrain implements Subsystem{
    // the subsystem wide variables
    boolean fieldRelative = true;
    SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        DTConstants.frontLeftLocation,
        DTConstants.frontRightLocation,
        DTConstants.backLeftLocation,
        DTConstants.backRightLocation
    );
    
    Pigeon2 gyro = new Pigeon2(DTConstants.gyroID);
    SwerveModule[] swerveModules = new SwerveModule[] { //fl,fr,bl,br
            new SwerveModule(DTConstants.frontLeftDriveID, DTConstants.frontLeftRotID, DTConstants.frontLeftEncoder, DTConstants.frontLeftOffset), 
            new SwerveModule(DTConstants.frontRightDriveID,DTConstants.frontRightRotID, DTConstants.frontRightEncoder, DTConstants.frontLeftOffset),
            new SwerveModule(DTConstants.backLeftDriveID,DTConstants.backLeftRotID, DTConstants.backLeftEncoder, DTConstants.frontLeftOffset), 
            new SwerveModule(DTConstants.backRightDriveID, DTConstants.backRightRotID, DTConstants.backLeftEncoder, DTConstants.frontLeftOffset),
    };
               
    public DriveTrain(){
    }

    public void driveJoystick(CommandPS5Controller driveController, boolean fieldRelative){
        double x = Math.pow(-MathUtil.applyDeadband(driveController.getLeftY(),0.001),3)*DTConstants.maxSpeed;
        double y = Math.pow(MathUtil.applyDeadband(driveController.getLeftX(),0.001),3)*DTConstants.maxSpeed;
        double rot = Math.pow(-MathUtil.applyDeadband(driveController.getRightX(),0.001),3)*DTConstants.maxRot;
        if(fieldRelative) {
            driveFieldRelative(x,y,rot);
        }
        else {
            driveRobotRelative(x,y,rot);
        }
    }

    public void driveRobotRelative(double x,double y,double rot){
        //chassis speeds based on drive controller
        ChassisSpeeds speed = new ChassisSpeeds(x,y,rot);
        driveFromChassisSpeeds(speed);
    }

    public void driveFieldRelative(double x,double y,double rot){
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(x,y,rot,gyro.getRotation2d());
        driveFromChassisSpeeds(speed);
    }

    /**
     * good luck
     * @param speed the chassis speeds object you get from another drive funtion
     */
    public void driveFromChassisSpeeds(ChassisSpeeds speed){

        //the module states from the chassis speed
        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(speed);
        
        for(int i = 0; i < 4;i++){
            swerveModules[i].fromModuleState(moduleStates[i]);
        } 
    }

    public void fieldRelativeToggle(){
        fieldRelative = !fieldRelative;
    }

    public Command driveCommand(){
        return Commands.runOnce(()->{driveJoystick(RobotContainer.driveController, fieldRelative);}, this);
    }
    
    public Command fieldRelToggleCommand(){
        return Commands.runOnce(()->{fieldRelativeToggle();});
    }
    
}