#ifndef FFW_FUNCTION_H
#define FFW_FUNCTION_H

namespace Motion {

struct FFWParameters
{
  double ka;
  double kv;
};

// feed-forward
class FFWFunction
{
  public:
    FFWFunction(FFWParameters parameters);

    double output(double acceleration, double velocity);

  private:
    FFWParameters parameters_;
};

}

#endif
