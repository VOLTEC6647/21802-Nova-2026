
package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import lombok.Getter;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Config
public class Vision extends SubsystemBase {

    private final Limelight3A camera;
    @Getter private LLResult result;

    public static double TURN_P = 0.05;



    private Bot bot;


    public Vision(Bot bot) {
        this.bot = bot;
        camera = bot.hMap.get(Limelight3A.class, "limelight");

        pipeline(1);
        initializeCamera();

    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }
    public void pipeline(int switchPipeline){
        camera.pipelineSwitch(switchPipeline);
    }

    public double getTurnPower() {
        if (!result.isValid()) {
            return 0.0;
        }
        double tx = getTx();

        return -tx * TURN_P;
    }

    public double getTx() {
        if (result == null) {
            return 0;
        }
        return result.getTx();
    }

    public double getTy() {
        if (result == null) {
            return 0;
        }
        return result.getTy();
    }



    @Override
    public void periodic() {
        result = camera.getLatestResult();

        if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    MecanumDrive.odo.setPosition(
                            new Pose2D(DistanceUnit.INCH,result.getBotpose().getPosition().x,
                                    result.getBotpose().getPosition().y,
                                    AngleUnit.RADIANS,
                                    result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)
                            )
                    );

            bot.telem.addData("Turn Power", getTurnPower());
            bot.telem.addData("Ty", getTy());
            bot.telem.addData("Tx", getTx());
            bot.telem.addData("Bot pose", botpose.toString());

            }
        }
    }
}
