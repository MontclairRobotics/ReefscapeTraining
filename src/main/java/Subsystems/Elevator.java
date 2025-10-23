package Subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemBase{
        public double speed;
        public static final double STARTING_HEIGHT = 0.98;
        public static final double MAX_EXTENSION = 1.22;
        public static final double MAX_HEIGHT = STARTING_HEIGHT + MAX_EXTENSION;
        private static final double ROTATIONS_PER_METER = 36 * 5 / 1000.0 * (1.0 / 9);
        private static final double METER_PER_ROTATIONS = 1.0/ROTATIONS_PER_METER;
        private static final double SLOW_DOWN_ZONE = 0;
        private static final double SLOWEST_SPEED = 0;
        public static final double MAX_VELO_RPS = 100.0;
        public static final double MAX_ACCELERATION_RPS = 350.0;

        private PIDController pidController;


        private final int LEFT_MOTOR_ID = 0;
        private final int RIGHT_MOTOR_ID = 0;
        public TalonFX rightTalonFX;
        public TalonFX leftTalonFX;

public Elevator (){
    rightTalonFX = new TalonFX (RIGHT_MOTOR_ID, "Drivetrain");
    leftTalonFX = new TalonFX (LEFT_MOTOR_ID, "Drivetrain");
    pidController = new PIDController(8.2697, 0, 0.068398);
    }

public void goToHeight (double height){
    double targetHeightFromRotations = height * ROTATIONS_PER_METER;
    rightTalonFX.getPosition();

}
}
