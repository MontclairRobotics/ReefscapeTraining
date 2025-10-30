package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

public class SwerveModule {
    TalonFX driveMotor;
    SparkMax turnMotor;
    /**
     * @param ID of the Drive Motor
     * @param ID of the Turn Motor 
     */
    public SwerveModule(int driveID, int turnID){
        driveMotor = new TalonFX(driveID);
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
