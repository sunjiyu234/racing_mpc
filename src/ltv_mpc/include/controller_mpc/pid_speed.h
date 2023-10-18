#ifndef PID_SPEED_H
#define PID_SPEED_H


class pid_speed
{
public:
    pid_speed(double kp, double ki, double kd);
    double pid_control(double error);
    ~pid_speed();

private:
    double pid_kp;
    double pid_ki;
    double pid_kd;
    double pre_error;
    double sum_error;
};

#endif // PID_SPEED_H
