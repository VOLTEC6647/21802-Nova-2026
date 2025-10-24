package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Vision;

public class Turret implements Subsystem {

    private final DcMotorEx turret;
    private final Vision vision;

    public static final int TURRET_LEFT_LIMIT = -400;
    public static final int TURRET_RIGHT_LIMIT = 400;


    public Turret(Bot bot) {
        vision = new Vision(bot);
        vision.register();

        turret = bot.hMap.get(DcMotorEx.class,"turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }


    @Override
    public void periodic(){
        setPower(vision.getTurnPower());

    }
    public void setPower(double power){

        int currentPosition = turret.getCurrentPosition();

        if (power < 0 && currentPosition <= TURRET_LEFT_LIMIT) {
            power = 0;
        } else if (power > 0 && currentPosition >= TURRET_RIGHT_LIMIT) {
            power = 0;
        }
        turret.setPower(power);


    }
}