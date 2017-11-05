
class Driver{

  private:
    // Motor A PWM Pins
    int MOTOR_A_CH_1;
    int MOTOR_A_CH_2;

    // Motor B PWM Pins
    int MOTOR_B_CH_1;
    int MOTOR_B_CH_2;

    // motor speed
    int TURNING_SPEED;

  public:
  Driver(int motorAlphaCh1, int motorAlphaCh2, int motorBetaCh1, int motorBetaCh2);
  void goFoward(int motorSpeed);
  void goBackward(int motorSpeed);
  void goLeft();
  void goRight();
  void halt();
};
