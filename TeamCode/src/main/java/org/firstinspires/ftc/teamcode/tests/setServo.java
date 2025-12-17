package org.firstinspires.ftc.teamcode.tests;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SetServo", group = "Tools")
public class setServo extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    Hood hood;

    public void initialize() {

        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);


        telem.addData("status","init");
        telem.update();

        hood = new Hood(hardwareMap, telemetry);
        hood.register();


    }

    @Config
    public static class targetServo{
        public static double hoodPos = 0;

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        hood.setPosition(targetServo.hoodPos);






        telem.addData("status","start");
    }
}