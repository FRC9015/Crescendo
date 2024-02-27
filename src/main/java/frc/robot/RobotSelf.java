package frc.robot;

//used so that selfdrive can be used in any file
public class RobotSelf {

    public static class RobotSelves{
        private static boolean AmpSelf = false;
        private static boolean SpeakerSelf = false;

        public static boolean getAmpSelf(){
            return AmpSelf;
        }
        public static void toggleAmpSelf(){
            AmpSelf = !AmpSelf;
        }

        public static boolean getSpeakerSelf(){
            return SpeakerSelf;
        }
        public static void toggleSpeakerSelf(){
            SpeakerSelf = !SpeakerSelf;
            System.out.println(SpeakerSelf);
        }

    }
}