package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.revrobotics.spark.SparkMax;

public class SwerveModule {
    SparkMax driveMotor;
    SparkMax turnMotor;
    public SwerveModule(int driveID, int turnID){
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
    }

}
