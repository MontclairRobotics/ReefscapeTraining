package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;

// if it has title case, its james, if it has no caps it's or (yes that's his name) and dashiell is also here in case anyone forgot
/**
 * the drivetrain subsystem.
 * length
 * 
 */
public class DriveTrain implements Subsystem{
    // the subsystem wide variables
    PIDController drivePID;
    PIDController rotPID;
    static boolean fieldRelative = true;
    SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        DTConstants.frontLeftLocation,
        DTConstants.frontRightLocation,
        DTConstants.backLeftLocation,
        DTConstants.backRightLocation
    );
        Pigeon2 gyro = new Pigeon2(DTConstants.gyroID);
    SwerveModule[] swerveModules = new SwerveModule[] { //fl,fr,bl,br
            new SwerveModule(DTConstants.frontLeftDriveID, DTConstants.frontLeftRotID), 
            new SwerveModule(DTConstants.frontRightDriveID,DTConstants.frontRightRotID),
            new SwerveModule(DTConstants.backLeftDriveID,DTConstants.backLeftRotID), 
            new SwerveModule(DTConstants.backRightDriveID, DTConstants.backRightRotID),
    };
               
    public DriveTrain(){
        drivePID = new PIDController(DTConstants.drkP,DTConstants.drkI,DTConstants.drkD);
        rotPID = new PIDController(DTConstants.rotkP,DTConstants.rotkI,DTConstants.rotkD);   
    }
    public void drive(CommandPS5Controller driveController, boolean fieldRelative){
        if(fieldRelative) {
            driveFieldRelative(driveController);
        }
        else {
            driveRobotRelative(driveController);
        }
    }
    public void driveRobotRelative(CommandPS5Controller driveController){
        //chassis speeds based on drive controller
        ChassisSpeeds speed = new ChassisSpeeds(
            Math.pow(-driveController.getLeftY(),3)*DTConstants.maxSpeed,
            Math.pow(driveController.getLeftX(),3)*DTConstants.maxSpeed,
            Math.pow(-driveController.getRightX(),3)*DTConstants.maxRot);
        driveFromChassisSpeeds(speed);
    }
    public void driveFieldRelative(CommandPS5Controller driveController){
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
            Math.pow(-driveController.getLeftY(),3)*DTConstants.maxSpeed,
            Math.pow(driveController.getLeftX(),3)*DTConstants.maxSpeed,
            Math.pow(-driveController.getRightX(),3)*DTConstants.maxRot,
            gyro.getRotation2d());
        driveFromChassisSpeeds(speed);
    }
    /**
     * good luck
     * @param speed the chassis speeds object you get from another drive funtion
     */
    public void driveFromChassisSpeeds(ChassisSpeeds speed){

        //the module states from the chassis speed
        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(speed);
        
        //I'm not sorry.
        for(int i = 0; i < 4;i++){
            swerveModules[i].driveWithVoltage(
                drivePID.calculate(
                    swerveModules[i].driveMotor.getMotorVoltage().getValueAsDouble(), //yes, the measurement and setpoint are different units.  
                    moduleStates[i].speedMetersPerSecond), //we're pretty sure it works. we hope it works.
                rotPID.calculate(
                    swerveModules[i].rotMotor.getAbsoluteEncoder().getPosition(),
                    moduleStates[i].angle.getRadians()));
        } 
    }
    public void fieldRelativeToggle(){
        fieldRelative = !fieldRelative;
    }

    public Command driveCommand(){
        return Commands.runOnce(()->{drive(RobotContainer.driverController, fieldRelative);}, this);
    }
    public Command fieldRelToggleCommand(){
        return Commands.runOnce(()->{fieldRelativeToggle();});
    }
    
}