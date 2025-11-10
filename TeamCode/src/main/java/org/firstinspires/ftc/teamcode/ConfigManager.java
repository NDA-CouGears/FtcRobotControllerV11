package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.*;
import org.json.JSONObject;

public class ConfigManager {
    private static final File CONFIG_FILE = AppUtil.getInstance().getSettingsFile("robot_config.json");

    // Example config data
    public double driveSpeed = 0.5;
    public double turnSpeed = 0.3;
    public String alliance = "RED";

    public void saveConfig() {
        try (FileWriter writer = new FileWriter(CONFIG_FILE)) {
            JSONObject json = new JSONObject();
            json.put("driveSpeed", driveSpeed);
            json.put("turnSpeed", turnSpeed);
            json.put("alliance", alliance);
            writer.write(json.toString(2)); // pretty print with indent
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void loadConfig() {
        if (!CONFIG_FILE.exists()) {
            return;
        }

        try (FileReader reader = new FileReader(CONFIG_FILE)) {
            char[] buffer = new char[(int) CONFIG_FILE.length()];
            reader.read(buffer);
            JSONObject json = new JSONObject(new String(buffer));

            driveSpeed = json.getDouble("driveSpeed");
            turnSpeed = json.getDouble("turnSpeed");
            alliance = json.getString("alliance");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
