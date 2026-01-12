package org.firstinspires.ftc.teamcode.experiment;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONObject;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class ConfigManager {
    private static final File CONFIG_FILE = AppUtil.getInstance().getSettingsFile("robot_config.json");

    public boolean testMode = true;
    public boolean blueAlliance = true;
    public boolean startNear = false;
    public boolean scanObelisk = true;
    public int startDelay = 0;
    public int intakeLine = 0;
    public int shootPos = 1;

    public void save() {
        try (FileWriter writer = new FileWriter(CONFIG_FILE)) {
            JSONObject json = new JSONObject();
            json.put("testMode", testMode);
            json.put("blueAlliance", blueAlliance);
            json.put("startNear", startNear);
            json.put("startDelay", startDelay);
            json.put("intakeLine", intakeLine);
            json.put("shootPos", shootPos);
            json.put("scanObelisk", scanObelisk);
            writer.write(json.toString(2)); // pretty print with indent
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void load() {
        if (!CONFIG_FILE.exists()) {
            return;
        }

        try (FileReader reader = new FileReader(CONFIG_FILE)) {
            char[] buffer = new char[(int) CONFIG_FILE.length()];
            reader.read(buffer);
            JSONObject json = new JSONObject(new String(buffer));

            testMode = json.getBoolean("testMode");
            blueAlliance = json.getBoolean("blueAlliance");
            startNear = json.getBoolean("startNear");
            startDelay = json.getInt("startDelay");
            if (json.has("intakeLine")) {
                intakeLine = json.getInt("intakeLine");
            }
            if (json.has("shootPos")) {
                shootPos = json.getInt("shootPos");
            }
            if (json.has("scanObelisk")) {
                scanObelisk = json.getBoolean("scanObelisk");
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    int curMenu = 0;
    boolean dpadPressed = false;

    public void displayMenu(Telemetry telemetry, Gamepad gamepad) {
        boolean changed = false;

        /*
        list of config items:
         - obelisk
         - shoot pos
         - allow intake line 0
         */

        // Display current menu state
        telemetry.addLine("dpad.y to select, dpad.x to change value");
        telemetry.addLine(String.format("%s alliance %s", curMenu == 0 ? "*" : "-", blueAlliance ? "BLUE" : "RED"));
        telemetry.addLine(String.format("%s start %s", curMenu == 1 ? "*" : "-", startNear ? "NEAR" : "FAR"));
        telemetry.addLine(String.format("%s delay %d", curMenu == 2 ? "*" : "-", startDelay));
        telemetry.addLine(String.format("%s testMode %b", curMenu == 3 ? "*" : "-", testMode));
        telemetry.addLine(String.format("%s intakeLine %d", curMenu == 4 ? "*" : "-", intakeLine));
        telemetry.addLine(String.format("%s shootPos %s", curMenu == 5 ? "*" : "-", (shootPos == 1)? "NEAR": "FAR"));
        telemetry.addLine(String.format("%s obelisk %s", curMenu == 6 ? "*" : "-", scanObelisk? "SCAN": "IGNORE"));


        // Do not allow another input after the dpad was pressed until it is released
        if (dpadPressed &&
                !gamepad.dpad_down &&
                !gamepad.dpad_up &&
                !gamepad.dpad_left &&
                !gamepad.dpad_right) {
            dpadPressed = false;
        }

        if (dpadPressed) {
            // We registered a press and it is still being held down, await a release
            return;
        }

        // Navigate between menu options
        if (gamepad.dpad_down && curMenu < 6) {
            curMenu++;
            dpadPressed = true;
        } else if (gamepad.dpad_up && curMenu > 0) {
            curMenu--;
            dpadPressed = true;
        }

        // Update selected value
        double delta = 0;
        if (gamepad.dpad_left) {
            delta = -1;
        } else if (gamepad.dpad_right) {
            delta = 1;
        }

        // Apply any changes
        if (delta != 0) {
            dpadPressed = true;
            changed = true;
            switch (curMenu) {
                case 0:
                    blueAlliance = !blueAlliance;
                    break;
                case 1:
                    startNear = !startNear;
                    shootPos = startNear ? 1: 2;
                    break;
                case 2:
                    startDelay += (int) delta;
                    if (startDelay < 0)
                        startDelay = 0;
                    break;
                case 3:
                    testMode = !testMode;
                    break;
                case 4:
                    intakeLine += (int) delta;
                    if (intakeLine > 3 || intakeLine < 0){
                        intakeLine = 0;
                    }
                    break;
                case 5:
                    shootPos = shootPos==1 ? 2:1;
                    break;
                case 6:
                    scanObelisk = !scanObelisk;
                    break;
            }
        }

        if (changed) {
            save();
        }
    }

}
