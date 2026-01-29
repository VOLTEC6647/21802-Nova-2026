package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Config
@Autonomous
public class AutoRed extends LinearOpMode {

    public static Pose startingPose = new Pose(109.56140233, 131.7055, Math.toRadians(0), PedroCoordinates.INSTANCE);
    double heading = 0;

    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        Follower f = Constants.createFollower(bot.hMap);
        f.setStartingPose(startingPose);
        f.update();

        Intake i = new Intake(hardwareMap, telem);
        i.register();

        Shooter s = new Shooter(hardwareMap, telem);
        s.register();

        Turret t = new Turret(hardwareMap, telem);
        t.resetEncoders();
        t.register();

        Indexer iN = new Indexer(hardwareMap, telem);
        iN.register();

        Stopper stopper = new Stopper(hardwareMap, telem);
        stopper.register();

        SequentialCommandGroup auto = new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(startingPose, new Pose(97.382, 100.4597)))
                                        .setLinearHeadingInterpolation(startingPose.getHeading(), heading)
                                        .build()
                                )
                        ),
                        // First Shooting Sequence (Replaced with Smart Logic)
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                // Runs for 4500ms, checking velocity every loop
                                new SmartFireCommand(s, i, iN, stopper, f, 4500)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(new Pose(97.382, 100.4597), new Pose(90, 82)))
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> { i.setPower(1); }),
                                new InstantCommand(() -> { f.setMaxPower(0.75); })
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(new Pose(90, 82), new Pose(125.000, 82)))
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(() -> { f.setMaxPower(1); })
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(new Pose(125.000, 82), new Pose(97.382, 100.4597)))
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                // Second Shooting Sequence (Replaced with Smart Logic)
                                new WaitCommand(750),
                                new SmartFireCommand(s, i, iN, stopper, f, 4500)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(new Pose(97.382, 100.4597), new Pose(90, 58.500)))
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> { i.setPower(1); }),
                                new InstantCommand(() -> { f.setMaxPower(0.75); })

                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(new Pose(90, 58.500), new Pose(133.000, 58.500)))
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(() -> { f.setMaxPower(1); })

                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(new Pose(133.000, 58.500), new Pose(90, 58.09090909090909)))
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(new Pose(90, 58.09090909090909), new Pose(97.382, 100.4597)))
                                        .setConstantHeadingInterpolation(heading)
                                        .build()
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                // Final Shooting Sequence (Replaced with Smart Logic)
                                new SmartFireCommand(s, i, iN, stopper, f, 4500)
                        )
                )
        );

        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            PoseStorage.currentPose = f.getPose();

            t.setTurretRED(f.getPose().getX(), f.getPose().getY(), f.getPose().getHeading());
            s.setVelocityRED(f.getPose().getX(), f.getPose().getY());
            telem.addData("POSE", f.getPose());
            telem.update();
        }
    }

    // --- Custom Command Class ---
    public class SmartFireCommand extends CommandBase {
        private final Shooter shooter;
        private final Intake intake;
        private final Indexer indexer;
        private final Stopper stopper;
        private final Follower follower;
        private final long duration;
        private final ElapsedTime timer = new ElapsedTime();

        public SmartFireCommand(Shooter s, Intake i, Indexer iN, Stopper stop, Follower f, long durationMs) {
            this.shooter = s;
            this.intake = i;
            this.indexer = iN;
            this.stopper = stop;
            this.follower = f;
            this.duration = durationMs;
            addRequirements(s, i, iN, stop);
        }

        @Override
        public void initialize() {
            timer.reset();
        }

        @Override
        public void execute() {
            double currentX = follower.getPose().getX();
            double currentY = follower.getPose().getY();

            double targetVel = shooter.tarVelRED(currentX, currentY);
            double currentVel = shooter.getCurrentVelocity();

            // The logic you requested:
            if (targetVel - Math.abs(currentVel) < 40) {
                // Velocity is good -> FIRE (Equivalent to "if g1.x")
                intake.setPower(0.93);
                stopper.setPosition(0); // Open
                indexer.setPower(1);    // Ensure indexer runs to push ring
            } else {
                // Velocity is bad -> WAIT/HOLD
                stopper.setPosition(1); // Closed
                // If velocity is bad, we usually stop feeding
                intake.setPower(0);
                indexer.setPower(0);
            }
        }

        @Override
        public boolean isFinished() {
            return timer.milliseconds() >= duration;
        }

        @Override
        public void end(boolean interrupted) {
            // Turn everything off when the command finishes
            intake.setPower(0);
            indexer.setPower(0);
            stopper.setPosition(1);
        }
    }
}