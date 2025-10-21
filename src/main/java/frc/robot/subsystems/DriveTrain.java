package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveTrain implements Subsystem{
    SwerveModule frontRight;      
    SwerveModule backRight;
    SwerveModule backLeft;
    SwerveModule frontLeft;
    
               
    public DriveTrain(){
        frontRight = new SwerveModule(-1,-1); //TODO: find the IDs???????
        frontLeft = new SwerveModule(-1,-1); //TODO: find the IDs???????
        backRight = new SwerveModule(-1,-1); //TODO: find the IDs???????
        backLeft = new SwerveModule(-1,-1); //TODO: find the IDs???????

    }
    

}
