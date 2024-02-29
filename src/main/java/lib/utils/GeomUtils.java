package lib.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class GeomUtils {
  public static Transform3d translationToTransform(Translation3d trans) {
    return new Transform3d(trans, new Rotation3d());
  }
}
