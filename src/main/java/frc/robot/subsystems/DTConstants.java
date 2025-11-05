package frc.robot.subsystems;

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
            static double drkP = -1; //TODO: FIND THESE
            static double drkI = -1; //TODO: FIND THESE
            static double drkD = -1; //TODO: FIND THESE
        //Rot
            static double rotkP = -1; //TODO: FIND THESE
            static double rotkI = -1; //TODO: FIND THESE
            static double rotkD = -1; //TODO: FIND THESE

    // Ports
            static int frontLeftDriveID = 1;
            static int frontLeftRotID = 2;
            static int frontRightDriveID = 3;
            static int frontRightRotID = 4;
            static int backLeftDriveID = 5;
            static int backLeftRotID = 6;
            static int backRightDriveID = 7;
            static int backRightRotID = 8;
            static int gyroID = 0;// TODO: confirm this
}