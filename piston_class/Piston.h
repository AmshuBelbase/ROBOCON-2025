#ifndef PISTON_H
#define PISTON_H


class Piston {
private:
  int extend;
  int retract;
  bool state;  // 1 - extended, 0 - retracted

public:
  Piston(int extend, int retract);
  void pistonControl(bool state);
  void extendPiston();
  void retractPiston();
  void toggle();
  void getStatus();
};

#endif