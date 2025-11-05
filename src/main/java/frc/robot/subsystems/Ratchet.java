package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ratchet extends SubsystemBase{
    public Servo rightServo;
    public Servo leftServo;
    public boolean isRatchetEngaged = false;

    public Ratchet(){
        leftServo = new Servo(8);
        rightServo = new Servo(6);
    }

    public void lockRatchet(){
        leftServo.set(1);
        rightServo.set(1);
        isRatchetEngaged = true;
    }

    public void unlockRatchet(){
        leftServo.set(0);
        rightServo.set(0);
        isRatchetEngaged = false;
    }

    public Command lockRatchetCommand(){
        return Commands.runOnce(() -> lockRatchet(), this);
    }

    public Command unlockRatchetCommand(){
        return Commands.runOnce(() -> unlockRatchet(), this);
    }

}