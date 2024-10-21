package XCYOS;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class FrameMagager {
   private ArrayList<Frame> allFrame = new ArrayList<>();

   //TODO
   Vector2d transform2World(Frame source, Vector2d position) {
      return null;
   }

   //TODO
   Vector2d transform(Frame source, Frame destination, Vector2d position) {
      return null;
   }

   public static class Frame {
      private Vector2d rel_to_world;
      private String name;
   }
}
