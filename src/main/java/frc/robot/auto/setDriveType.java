package frc.robot.auto;

import frc.robot.Globals;

public class setDriveType {
    public void setType(int type) {
        switch (type) {
            case 1:
                Globals.GlobalVars.leftSpeaker = false;
                Globals.GlobalVars.rightSpeaker = false;
                Globals.GlobalVars. middleSpeaker = true;
                break;
            case 2:
                Globals.GlobalVars.leftSpeaker = true;
                Globals.GlobalVars.rightSpeaker = false;
                Globals.GlobalVars.middleSpeaker = false;
                break;
            case 3:
                Globals.GlobalVars.leftSpeaker = false;
                Globals.GlobalVars.rightSpeaker = true;
                Globals.GlobalVars. middleSpeaker = false;
                break;
            default:
                Globals.GlobalVars.leftSpeaker = false;
                Globals.GlobalVars.rightSpeaker = false;
                Globals.GlobalVars.middleSpeaker = true;
                break;
        }
        return;
    }
}
