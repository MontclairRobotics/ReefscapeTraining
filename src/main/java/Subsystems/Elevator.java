package Subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

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
        public double rightDisplacement;
        public double leftDisplacement;
        public double averageDisplacement;

        private PIDController pidController;


        private final int LEFT_MOTOR_ID = 20;
        private final int RIGHT_MOTOR_ID = 21;
        public TalonFX rightTalonFX;
        public TalonFX leftTalonFX;

public Elevator (){
    rightTalonFX = new TalonFX (RIGHT_MOTOR_ID, "Drivetrain");
    leftTalonFX = new TalonFX (LEFT_MOTOR_ID, "Drivetrain");
    pidController = new PIDController(8.2697, 0, 0.068398);
    }

private double getExtension(){
    double rightDisplacement = (rightTalonFX.getPosition().getValueAsDouble());
    double leftDisplacement = (rightTalonFX.getPosition().getValueAsDouble());
    double averageDisplacement = ((rightDisplacement + leftDisplacement)/2.0);
    return averageDisplacement * METERS_PER_ROTATION;

}
public void goToExtension (double targetExtension){
    double pidOutput = pidController.calculate(getExtension(), targetExtension);
    rightTalonFX.setVoltage(MathUtil.clamp(pidOutput, -12.0, 12.0));
    leftTalonFX.setVoltage(MathUtil.clamp(pidOutput, -12.0, 12.0));
}

public void stop(){
    leftTalonFX.setVoltage(0);
    rightTalonFX.setVoltage(0);
}

public boolean isAtSafeHeight(){
    if (getExtension() == MathUtil.applyDeadband(MAX_HEIGHT, 0.1)||
    getExtension()==MathUtil.applyDeadband(STARTING_HEIGHT, 0.1)){
        return false;
    }
    else{
        return true;
    }
}

public void manualControl(){
    double speed = Math.pow((RobotContainer.operatorController.getLeftY()), 3);
    double percentExtension = this.getExtension()/MAX_EXTENSION;
    if (percentExtension >= (1-SLOW_DOWN_ZONE)||percentExtension <= SLOW_DOWN_ZONE){
        if (percentExtension >= 0.99|| percentExtension <= 0.01){
            stop();
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
    .until(() -> getExtension() == MathUtil.applyDeadband(targetExtension, 0.01));
}

public Command manualContralCommand (){
    return Commands.run(() -> manualControl(), this);
    }
}