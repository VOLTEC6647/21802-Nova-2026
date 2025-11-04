package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    @Config
    public static class ShooterPIDF{
        public static double kp = 1;
        public static double ki = 0;
        public static double kd = 0.05;
        public static double kf = -0.005;
    }
    public static double targetVelocity = -1305;
    private MultipleTelemetry telemetry;



    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        motor1.setVelocityPIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf
        );


    }

    @Override
    public void periodic(){
        double currentVelocity = motor1.getVelocity();

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);


    }
    public void setVelocity(){
        motor1.setVelocity(targetVelocity);
        motor2.setVelocity(targetVelocity);


    }
}