public class PIDController
{
    public float Kp, Ki, Kd;
    private float integral;
    private float lastError;

    public PIDController(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        integral = 0;
        lastError = 0;
    }

    public float UpdatePID(float setpoint, float measured, float deltaTime)
    {
        float error = setpoint - measured;
        integral += error * deltaTime;
        float derivative = (error - lastError) / deltaTime;
        lastError = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    public void Reset()
    {
        integral = 0;
        lastError = 0;
    }
}
