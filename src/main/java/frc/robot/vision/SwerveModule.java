public class SwerveModule extends SubsystemBase {
    Translation2d pose;
    int id;
    public SwerveModule(Translation2d pose, int id) {
        this.pose = pose;
        this.id = id;
    }


}