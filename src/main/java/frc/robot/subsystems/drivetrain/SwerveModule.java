package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

public class SwerveModule {
    TalonFX driveMotor;
    SparkMax rotMotor;
    PIDController drivePID;
    PIDController rotPID;
    double encoderOffset;
    CANcoder encoder;
        /**
         * @param ID of the Drive Motor
         * @param ID of the Rotation Motor 
         */
        public SwerveModule(int driveID, int rotID,int encoderID, double offset){
            driveMotor = new TalonFX(driveID);
            rotMotor = new SparkMax(rotID, MotorType.kBrushless);
            drivePID = new PIDController(DTConstants.drkP,DTConstants.drkI,DTConstants.drkD);
            rotPID = new PIDController(DTConstants.rotkP,DTConstants.rotkI,DTConstants.rotkD);
            rotPID.enableContinuousInput(-Math.PI, Math.PI);
            encoder = new CANcoder(encoderID);
            encoderOffset = offset;
        }
    
        /**
         * Sets The Voltage Of A Drive Motor and a Rotation Motor 
         */
        public void driveWithVoltage(double driveVoltage, double rotVoltage){
            driveMotor.setVoltage(driveVoltage);
            rotMotor.setVoltage(rotVoltage);
        }
        public void fromModuleState(SwerveModuleState moduleState) {
            // encoder gives position as rotation, so we have to multiply by 2PI to get to radians.
            double currentRot = encoder.getPosition().getValue().magnitude()*2*Math.PI+encoderOffset;
            moduleState.optimize(Rotation2d.fromRadians(currentRot));
            driveWithVoltage(
                    drivePID.calculate(driveMotor.getVelocity().getValue().magnitude()*0.407, // velocity is in rotations per second,
                                        moduleState.speedMetersPerSecond),                    // so we have to multiply by a conversion factor.
                    rotPID.calculate(currentRot, moduleState.angle.getRadians()));
    }
}
