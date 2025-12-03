// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.GamePiece;
import frc.robot.util.PoseUtils;
import frc.robot.util.TagOffset;
import frc.robot.util.TunerConstants;
import frc.robot.vision.Limelight;

public class RobotContainer {

  public static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public static CommandPS5Controller driveController = new CommandPS5Controller(0);
  public static DriveTrain driveTrain = new DriveTrain();
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static Elevator elevator = new Elevator();
  //public final NetworkTable networkTable;

  

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    driveTrain.setDefaultCommand(driveTrain.driveCommand());
    driveController.square().onTrue(driveTrain.fieldRelToggleCommand());

    elevator.setDefaultCommand(elevator.manualContralCommand());
    operatorController.L1().onTrue(elevator.goToExtensionCommand(Elevator.L1_HEIGHT));
    operatorController.L2().onTrue(elevator.goToExtensionCommand(Elevator.L2_HEIGHT));
    operatorController.R1().onTrue(elevator.goToExtensionCommand(Elevator.L3_HEIGHT));
    operatorController.R2().onTrue(elevator.goToExtensionCommand(Elevator.L4_HEIGHT));

  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}