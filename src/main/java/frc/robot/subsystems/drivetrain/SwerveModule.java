package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;


public class SwerveModule extends SubsystemBase {
    TalonFX driveMotor;
    SparkMax rotMotor;
    PIDController drivePID;
    PIDController rotPID;

    double encoderOffset;
    CANcoder encoder;

    NetworkTable table;
    NetworkTable angleTable;
    NetworkTable speedTable;
    DoubleTopic speedSetpointTop;
    DoubleTopic speedMeasureTop;
    DoubleTopic speedVoltageTop;
    DoubleTopic angleSetpointTop;
    DoubleTopic angleMeasureTop;
    
    /**
     * @param ID of the Drive Motor
     * @param ID of the Rotation Motor 
     */
    public SwerveModule(int driveID, int rotID,int encoderID, double offset, NetworkTable netTable){
        driveMotor = new TalonFX(driveID);
        rotMotor = new SparkMax(rotID, MotorType.kBrushless);
        drivePID = new PIDController(DTConstants.drkP,DTConstants.drkI,DTConstants.drkD);
        rotPID = new PIDController(DTConstants.rotkP,DTConstants.rotkI,DTConstants.rotkD);
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        encoder = new CANcoder(encoderID);
        encoderOffset = offset;
        
        table = netTable;
            
        angleTable = table.getSubTable("Angle");
        speedTable = table.getSubTable("Speed");
        
        speedSetpointTop =  speedTable.getDoubleTopic("setpoint");
        speedMeasureTop =  speedTable.getDoubleTopic("measure");
        speedVoltageTop = speedTable.getDoubleTopic("voltage");
        
        angleSetpointTop =  angleTable.getDoubleTopic("setpoint");
        angleMeasureTop =  angleTable.getDoubleTopic("measure");
    }

    /**
     * Sets The Voltage Of A Drive Motor and a Rotation Motor 
     */
    public void logging(double speedMeasure,double speedSetpoint, double speedVoltage, double angleMeasure, double angleSetpoint) {

        speedSetpointTop.publish().set(speedSetpoint);
        speedMeasureTop.publish().set(speedMeasure);
        speedVoltageTop.publish().set(speedVoltage);

        angleSetpointTop.publish().set(angleSetpoint);
        angleMeasureTop.publish().set(angleMeasure);
    }

    public void driveWithVoltage(double driveVoltage, double rotVoltage){
        driveMotor.setVoltage(driveVoltage);
        rotMotor.setVoltage(rotVoltage);
    }

    public void fromModuleState(SwerveModuleState moduleState) {
        // encoder gives position as rotation, so we have to multiply by 2PI to get to radians.
        double currentRot = encoder.getPosition().getValue().magnitude()*2*Math.PI+encoderOffset;
        double currentVelocity = driveMotor.getVelocity().getValue().magnitude()*0.407; // velocity is in rotations per second, so we have to multiply by a conversion
        double currentVoltage = driveMotor.getMotorVoltage().getValueAsDouble();

        moduleState.optimize(Rotation2d.fromRadians(currentRot));

        logging(currentVelocity,moduleState.speedMetersPerSecond,currentVoltage,currentRot,moduleState.angle.getRadians());

        driveWithVoltage(
                drivePID.calculate(currentVelocity, moduleState.speedMetersPerSecond),                    
                rotPID.calculate(currentRot, moduleState.angle.getRadians()));
    }
}