package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Vision;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.io.File;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends CommandOpMode {
    private Bot bot;
    private Vision vision;
    private Follower follower;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;
    private Shooter shooter;
    private Intake intake;
    private Indexer indexer;
    private Turret turret;


    public void initialize() {


        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();


        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // drive region

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);
        bot.getImu().resetYaw();

        File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        String team = ReadWriteFile.readFile(myFileName);


        if (team.equals("blue")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }
        if (team.equals("red")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }
        vision = new Vision(bot);
        vision.register();

        shooter = new Shooter(bot);
        shooter.register();

        intake = new Intake(bot);
        intake.register();

        indexer = new Indexer(bot);
        indexer.register();

        drive = new MecanumDrive(bot);
        drive.register();

        turret = new Turret(bot);
        turret.register();


        TeleopDriveCommand driveCommand = new TeleopDriveCommand(
                drive,
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                ()-> driverGamepad.getRightX(),
                ()-> bot.speed,
                ()-> bot.rotMultiplier

        );
        drive.setDefaultCommand(driveCommand);
        bot.speed = 1;



        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whileHeld(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> shooter.setVelocity()),
                                new InstantCommand(()-> intake.setPower(1))
                        )
                );
        new GamepadButton(
                driverGamepad, GamepadKeys.Button.A)
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> shooter.setVelocity()),
                                new InstantCommand(()-> intake.setPower(0))

                        )
                );

        //INDEXER
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whileHeld(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> indexer.setPower(1))
                        )
                );
        new GamepadButton(
                driverGamepad, GamepadKeys.Button.X)
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> indexer.setPower(0))
                        )
                );

        while (opModeInInit()){
            telem.update();
        }


    }
    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telem.update();
    }

}