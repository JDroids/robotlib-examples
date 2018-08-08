package org.firstinspires.ftc.teamcode.java;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.java.robot.Robot;
import org.firstinspires.ftc.teamcode.java.robot.commands.StayInPlace;

/**
 * This represents a simple Java autonomous that tries to stay in the same spot for the whole
 * autonomous period.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SimpleAutoJava")
public class Autonomous extends OpMode{
    private final StayInPlace stayInPlace = new StayInPlace();

    @Override
    public void init() {
        Robot.getInstance().autonomousInit();
    }

    @Override
    public void start() {
        stayInPlace.start();
    }

    @Override
    public void loop() {}

    @Override
    public void stop() {
        stayInPlace.cancel();
    }
}
