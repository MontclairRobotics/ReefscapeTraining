package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkMax;

public class SwerveModule {
    SparkMax driveMotor;
    SparkMax turnMotor;
    /**
     * @param ID of the Drive Motor
     * @param ID of the Turn Motor 
     */
    public SwerveModule(int driveID, int turnID){
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
    }
    /**
     * Sets The Voltage Of A Drive Motor and a Turn Motor 
     */
    public void driveWithVoltage(double driveVoltage, double rotVoltage){
        driveMotor.setVoltage(driveVoltage);
        turnMotor.setVoltage(rotVoltage);
    }
}
