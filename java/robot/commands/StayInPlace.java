package org.firstinspires.ftc.teamcode.java.robot.commands;

import com.jdroids.robotlib.command.Command;
import com.jdroids.robotlib.pid.PIDOutput;
import com.jdroids.robotlib.pid.PIDSource;
import com.jdroids.robotlib.pid.SimplePIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.java.robot.Robot;
import org.firstinspires.ftc.teamcode.java.robot.subsystems.Drive;
import org.jetbrains.annotations.NotNull;

/**
 * This is a simple Command demonstrating how to use a Command in Java, along with how to use
 * a SimplePIDController.
 */
public class StayInPlace extends Command {
    public StayInPlace() {
        requires(Robot.getInstance().drive);

        sources = new MotorPositionPIDSource[]{new MotorPositionPIDSource(Drive.Motor.LEFT),
                new MotorPositionPIDSource(Drive.Motor.RIGHT)};

        outputs = new MotorPowerPIDOutput[]{new MotorPowerPIDOutput(Drive.Motor.LEFT),
                new MotorPowerPIDOutput(Drive.Motor.RIGHT)};

        //These numbers are samples and not representative of any actual PID coefficients
        coefficients = new PIDCoefficients[]{new PIDCoefficients(0.43, 0.2, 0.6),
                new PIDCoefficients(0.24, 0.4, 0.3)};

        targetPositions = new int[2];
    }

    private Drive drive = Robot.getInstance().drive;
    private Drive.DriveSignal signal = new Drive.DriveSignal(0.0, 0.0);

    private MotorPositionPIDSource sources[];
    private MotorPowerPIDOutput outputs[];

    private PIDCoefficients coefficients[];

    private SimplePIDController controllers[];

    private int targetPositions[];

    private class MotorPositionPIDSource implements PIDSource {
        Drive.Motor motor;

        MotorPositionPIDSource(Drive.Motor motor) {
            this.motor = motor;
        }

        @Override
        public PIDSource.Type getPIDSourceType() {
            return Type.DISPLACEMENT;
        }

        @Override
        public void setPIDSourceType(@NotNull PIDSource.Type type) {}

        @Override
        public double pidGet() {
            return (double)(drive.getPosition(motor));
        }
    }

    private class MotorPowerPIDOutput implements PIDOutput {
        Drive.Motor motor;

        MotorPowerPIDOutput(Drive.Motor motor) {
            this.motor = motor;
        }

        @Override
        public void pidWrite(double output) {
            switch (motor) {
                case LEFT:
                    signal.leftPower = output;
                case RIGHT:
                    signal.rightPower = output;
            }
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
        targetPositions[0] = drive.getLeftPosition();
        targetPositions[1] = drive.getRightPosition();

        for (int i = 0; i <= 1; ++i) {
            controllers[i] = new SimplePIDController(sources[i], outputs[i], coefficients[i],
                    0.0, 0.0);
            controllers[i].setSetpoint(i);
        }
    }

    @Override
    public void execute() {
        for (SimplePIDController controller : controllers) {
            controller.calculate();
        }

        drive.driveOpenLoop(signal);
    }

}
