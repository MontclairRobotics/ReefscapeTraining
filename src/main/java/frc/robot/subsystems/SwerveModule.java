package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

public class SwerveModule {
    TalonFX driveMotor;
    SparkMax rotMotor;
    /**
     * @param ID of the Drive Motor
     * @param ID of the Rotation Motor 
     */
    public SwerveModule(int driveID, int rotID){
        driveMotor = new TalonFX(driveID);
        rotMotor = new SparkMax(rotID, MotorType.kBrushless);
    }
    /**
     * Sets The Voltage Of A Drive Motor and a Rotation Motor 
     */
    public void driveWithVoltage(double driveVoltage, double rotVoltage){
        driveMotor.setVoltage(driveVoltage);
        rotMotor.setVoltage(rotVoltage);
    }
}
