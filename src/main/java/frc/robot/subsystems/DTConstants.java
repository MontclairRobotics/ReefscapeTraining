package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class DTConstants{
    // MODULE LOCATIONS
    static Translation2d frontLeftLocation = new Translation2d(0.381, 0.381); //TODO: find the measurement????????????
    static Translation2d frontRightLocation = new Translation2d(0.381, -0.381); 
    static Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    static Translation2d backRightLocation = new Translation2d(-0.381, -0.381);
    // MAX SPEEDS
    static double maxSpeed = 10;//TODO: find max speed
    static double maxRotation = 10; //TODO: find max rotation speed
    // PIDS
        //DRIVE
            static double drkP = -1; //TODO: FIND THESE
            static double drkI = -1; //TODO: FIND THESE
            static double drkD = -1; //TODO: FIND THESE
        //ROTATION
            static double rotkP = -1; //TODO: FIND THESE
            static double rotkI = -1; //TODO: FIND THESE
            static double rotkD = -1; //TODO: FIND THESE

    // Ports
            static int frontLeftDriveID = -1;
            static int frontLeftRotationID = -1;
            static int frontRightDriveID = -1;
            static int frontRightRotationID = -1;
            static int backLeftDriveID = -1;
            static int backLeftRotationID = -1;
            static int backRightDriveID = -1;
            static int backRightRotationID = -1;
}