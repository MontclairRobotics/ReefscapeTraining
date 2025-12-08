package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;

public class DTConstants{
    // MODULE LOCATIONS
        static Translation2d frontLeftLocation = new Translation2d(0.307594, 0.307594);
        static Translation2d frontRightLocation = new Translation2d(0.307594, -0.307594); 
        static Translation2d backLeftLocation = new Translation2d(-0.307594, 0.307594);
        static Translation2d backRightLocation = new Translation2d(-0.307594, -0.307594);
    // MAX SPEEDS
        static double maxSpeed = 10;//TODO: find max speed METERS PER SECOND
        static double maxRot = 10; //TODO: find max rotation speed
    
    // PIDS
        //DRIVE
            static double drkP = 0; //TODO: FIND THESE
            static double drkI = 0; 
            static double drkD = 0; 
        //ROT
            static double rotkP = 0; //TODO: FIND THESE
            static double rotkI = 0; 
            static double rotkD = 0; 
    // ENCODER OFFSETS
        static double frontLeftOffset = -1;//TODO: FIND THESE
        static double frontRightOffset = -1;
        static double backLeftOffset = -1; 
        static double backRightOffset = -1; 

    // PORTS
        static int frontLeftDriveID = 1;
        static int frontLeftRotID = 2;
        static int frontRightDriveID = 3;
        static int frontRightRotID = 4;
        static int backLeftDriveID = 5;
        static int backLeftRotID = 6;
        static int backRightDriveID = 7;
        static int backRightRotID = 8;
        static int gyroID = 0;// TODO: confirm this
        // ENCODERS
        static int frontLeftEncoder = 0; //TODO: find these
        static int frontRightEncoder = 0;
        static int backLeftEncoder = 0;
        static int backRightEncoder = 0; 


}