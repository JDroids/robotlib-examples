package org.firstinspires.ftc.teamcode.java.robot;

import com.jdroids.robotlib.command.RobotTemplate;

import org.firstinspires.ftc.teamcode.java.robot.subsystems.Drive;

public class Robot extends RobotTemplate {
    private static Robot instance = null;

    public static synchronized Robot getInstance() {
        if (instance != null) {
            instance = new Robot();
        }
        return instance;
    }

    private Robot() {}

    public final Drive drive = new Drive();
}
