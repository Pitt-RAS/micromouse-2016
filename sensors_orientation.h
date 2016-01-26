#ifndef MICROMOUSE_SENSORS_ORIENTATION_H_
#define MICROMOUSE_SENSORS_ORIENTATION_H_

class Orientation {
  private:
    Orientation();
    static void interruptHandler();

//    Orientation* _instance = NULL;
  public:
    Orientation* getInstance();

    void update();

    void resetHeading();
    float getHeading();
    
    void resetMaxForwardAccel();
    void resetMaxRadialAccel();
    float getMaxForwardAccel();
    float getMaxRadialAccel();
};

#endif
