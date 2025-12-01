package Subsystems;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import frc.robot.util.LimitSwitch;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends SubsystemBase{

        public static final double STARTING_HEIGHT = 0.98;
        public static final double MAX_EXTENSION = 1.22;
        public static final double MAX_HEIGHT = STARTING_HEIGHT + MAX_EXTENSION;
        private static final double METERS_PER_ROTATION = 36 * 5 / 1000.0 * (1.0 / 9);
        private static final double ROTATIONS_PER_METER = 1.0/METERS_PER_ROTATION;
        private static final double SLOW_DOWN_ZONE = 0.07;
        private static final double SLOWEST_SPEED = 0.5;
        public static final double MAX_VELO_RPS = 100.0;
        public static final double MAX_ACCELERATION_RPS = 350.0;
        public static final double L1_HEIGHT = 0;
        public static final double L2_HEIGHT = 0.025;
        public static final double L3_HEIGHT = 0.495;
        public static final double L4_HEIGHT = 1.2;
        public static final double CLIMB_HEIGHT = 0;

        public double speed;
        public double pidOutput;
        public double ffOutput;
        public double totalOutput;
        public double rightDisplacement;
        public double leftDisplacement;
        public double averageDisplacement;
        public double targetExtension;
        public double desiredVelocity;
        public double percentExtension;

        private PIDController pidController;
        private ElevatorFeedforward feedForward;

        private final int LEFT_MOTOR_ID = 20;
        private final int RIGHT_MOTOR_ID = 21;
        public TalonFX rightTalonFX;
        public TalonFX leftTalonFX;

        private final DoublePublisher speedPub;
        private final DoublePublisher currentExtensionPub;
        private final DoublePublisher targetExtensionPub;
        private final DoublePublisher pidOutputPub;
        private final DoublePublisher ffOutputPub;
        private final DoublePublisher totalOutputPub;
        private final DoublePublisher rightMotorDisplacementPub;
        private final DoublePublisher leftMotorDisplacementPub;
        public final DoublePublisher percentExtensionPub;

public Elevator (){
    rightTalonFX = new TalonFX (RIGHT_MOTOR_ID, "Drivetrain");
    leftTalonFX = new TalonFX (LEFT_MOTOR_ID, "Drivetrain");
    rightTalonFX.setNeutralMode(NeutralModeValue.Brake);

    pidController = new PIDController(8.2697, 0, 0.068398);
    feedForward = new ElevatorFeedforward(0.058548, 0.22, 0.10758);
    feedForward = new ElevatorFeedforward(0.058548, 0.22, 0.10758);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable elevatorNetworkTable = inst.getTable("Elevator");
    speedPub = elevatorNetworkTable.getDoubleTopic("Current speed").publish();
    currentExtensionPub = elevatorNetworkTable.getDoubleTopic("Current extension(m)").publish();
    targetExtensionPub = elevatorNetworkTable.getDoubleTopic("Current target extension (m)").publish();
    pidOutputPub = elevatorNetworkTable.getDoubleTopic("Current PID Output (V)").publish();
    ffOutputPub = elevatorNetworkTable.getDoubleTopic("Current feed forward output (V)").publish();
    totalOutputPub = elevatorNetworkTable.getDoubleTopic("Current total output (V)").publish();
    rightMotorDisplacementPub = elevatorNetworkTable.getDoubleTopic("Current average displacement of the right motor (rot)").publish();
    leftMotorDisplacementPub = elevatorNetworkTable.getDoubleTopic("Current average displacement of the left motor (rot)").publish();
    percentExtensionPub = elevatorNetworkTable.getDoubleTopic("Current percent extension").publish();

    pidController.setTolerance(0.1);
    }

private double getExtension(){
    rightDisplacement = rightTalonFX.getPosition().getValueAsDouble();
    leftDisplacement = leftTalonFX.getPosition().getValueAsDouble();
    averageDisplacement = (rightDisplacement + leftDisplacement) / 2.0;
    return averageDisplacement * METERS_PER_ROTATION;
}


public void goToExtension (double targetExtension){
    this.targetExtension = targetExtension;
    pidOutput = pidController.calculate(getExtension(), targetExtension);
    desiredVelocity = 0.0;
    ffOutput = feedForward.calculate(desiredVelocity);
    totalOutput = pidOutput + ffOutput;
    rightTalonFX.setVoltage(MathUtil.clamp(totalOutput, -12.0, 12.0));
    leftTalonFX.setVoltage(MathUtil.clamp(totalOutput, -12.0, 12.0));
    
}

public void stop(){
    leftTalonFX.setVoltage(0);
    rightTalonFX.setVoltage(0);
}

public boolean isNearTopOfElevator(){
    if (Math.abs(MAX_EXTENSION-getExtension())<=0.1){
        return true;
    }
    else {
        return false;
    }
}

public boolean isNearBottomOfElevator(){
    if (Math.abs(getExtension()-STARTING_HEIGHT)<=0.1){
        return true;
    }
    else {
        return false;
    }
}


public void manualControl(){
    double speed = MathUtil.applyDeadband(Math.pow((RobotContainer.operatorController.getLeftY()), 3), 0.05);
    double percentExtension = this.getExtension()/MAX_EXTENSION;
    if (percentExtension >= (1-SLOW_DOWN_ZONE)||percentExtension <= SLOW_DOWN_ZONE){
        if (isNearTopOfElevator()){
            leftTalonFX.set(Math.min(0.0, speed));
            rightTalonFX.set(Math.min(0.0, speed));
        }
        else if (isNearBottomOfElevator()){
            leftTalonFX.set(Math.max(0.0, speed));
            rightTalonFX.set(Math.max(0.0, speed));
        }
        else{
            leftTalonFX.set(Math.min(speed, SLOWEST_SPEED));
            rightTalonFX.set(Math.min(speed, SLOWEST_SPEED));
        }
    }
    else {
        leftTalonFX.set(speed);
        rightTalonFX.set(speed); 
    }
}

public Command goToExtensionCommand (double targetExtension){
    return Commands.run(() -> goToExtension(targetExtension), this)
    .until(() -> pidController.atSetpoint());
}

public Command manualContralCommand (){
    return Commands.run(() -> manualControl(), this);
    }

@Override
public void periodic(){
    currentExtensionPub.set(getExtension());
    targetExtensionPub.set(targetExtension);
    pidOutputPub.set(pidOutput);
    ffOutputPub.set(ffOutput);
    totalOutputPub.set(totalOutput);
    rightMotorDisplacementPub.set(rightDisplacement);
    leftMotorDisplacementPub.set(leftDisplacement);
    percentExtensionPub.set(percentExtension);
    speedPub.set(speed);
    }
}