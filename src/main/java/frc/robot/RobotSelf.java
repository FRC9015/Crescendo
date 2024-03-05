package frc.robot;

//used so that selfdrive can be used in any file
public class RobotSelf {

    public static class RobotSelves{
        private static boolean AmpSelf = false;
        private static boolean SpeakerSelf = false;
        private static boolean intakeSelf = false;
        private static boolean SubWooferSelf = false;
        private static boolean AmpPresetSelf = false;
        private static boolean flatSelf = false;


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
            
        }

        public static boolean getIntakeSelf(){
            return intakeSelf;
            
        }
        public static void toggleIntakeSelf(){
            if(!SubWooferSelf && !AmpPresetSelf && !flatSelf){
                intakeSelf = !intakeSelf;
            }    
        }
        
        public static boolean getSubWooferSelf(){
            return SubWooferSelf;
        }
        public static void toggleSubWooferSelf(){
            if(!intakeSelf && !AmpPresetSelf && !flatSelf){
                SubWooferSelf = !SubWooferSelf;
            }  
           
        }

        public static boolean getAmpPrestSelf(){
            return AmpPresetSelf;
        }
        public static void toggleAmpPresetSelf(){
            if(!SubWooferSelf && !intakeSelf && !flatSelf){
                AmpPresetSelf = !AmpPresetSelf;
            }  
            
        }

        public static boolean getFlatSelf(){
            return flatSelf;
        }
        public static void toggleFlatSelf(){
            if(!SubWooferSelf && !AmpPresetSelf && !intakeSelf){
                flatSelf = !flatSelf;
            }  
            
        }
    }
}