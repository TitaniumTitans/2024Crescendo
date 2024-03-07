package frc.robot.subsystems.vision;

public class PhotonVisionIOInputs {
  protected boolean camConnected = false;
  protected double camLatency = 0.0;
  protected boolean tagsFound = false;
  protected int[] tagIds = new int[] {};
  protected int numTagsFound = 0;

  public boolean isCamConnected() {
    return camConnected;
  }

  public void setCamConnected(boolean camConnected) {
    this.camConnected = camConnected;
  }

  public double getCamLatency() {
    return camLatency;
  }

  public void setCamLatency(double camLatency) {
    this.camLatency = camLatency;
  }

  public int[] getTagIds() {
    return tagIds;
  }

  public void setTagIds(int[] tagIds) {
    this.tagIds = tagIds;
  }

  public int getNumTagsFound() {
    return numTagsFound;
  }

  public void setNumTagsFound(int numTagsFound) {
    this.numTagsFound = numTagsFound;
  }

  public boolean isTagsFound() {
    return tagsFound;
  }

  public void setTagsFound(boolean tagsFound) {
    this.tagsFound = tagsFound;
  }
}
