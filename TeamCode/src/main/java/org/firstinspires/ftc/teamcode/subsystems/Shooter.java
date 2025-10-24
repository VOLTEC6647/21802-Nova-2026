package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    @Config
    public static class ShooterPIDF{
        public static double kp = 0.009;
        static double ki = 0;
        static double kd = 0;
        public static double kf = 0;
    }
    public static double targetVelocity = 6000;
    private Bot bot;


    public Shooter(Bot bot) {
        this.bot = bot;

        motor1 = bot.hMap.get(DcMotorEx.class, "motor1");
        motor2 = bot.hMap.get(DcMotorEx.class, "motor2");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        motor1.setVelocityPIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf
        );

        motor2.setVelocityPIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf
        );

    }

    @Override
    public void periodic(){
        double currentVelocity = motor1.getVelocity();

        bot.telem.addData("Target Velocity", targetVelocity);
        bot.telem.addData("Current Velocity", currentVelocity);


    }
    public void setVelocity(){
        motor1.setVelocity(targetVelocity);
        motor2.setVelocity(targetVelocity);


    }
}