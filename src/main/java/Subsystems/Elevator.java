package Subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemBase{
        int speed;
        private final int LEFT_MOTOR_ID = 1;
        private final int RIGHT_MOTOR_ID = 1;
        public TalonFX rightTalonFX;
        public TalonFX leftTalonFX;

public Elevator (){
    rightTalonFX = new TalonFX (RIGHT_MOTOR_ID, "Drivetrain");
    leftTalonFX = new TalonFX (LEFT_MOTOR_ID, "Drivetrain");
    }
}
