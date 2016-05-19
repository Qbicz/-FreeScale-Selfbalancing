#ifndef _DIGITALPID_H_
#define _DIGITALPID_H_

// PID
float StabilP = 0.2; //25;
float StabilI = 0.5;
float StabilD = 0.15;
//const float StabilTp = StabilRate/1000000;
float dt = 0.01;

Timer PDTimer;
int StabilRate = 10000; // 100Hz

static const float MAX_ERROR =  40.0;   // Iterm windup constants
static const float ITERM_MAX = 500.0;

// declarations
float constrain(float input, float min, float max);

class DigitalPID
{
        float setPoint;
        float errorOld;      // Last error
        float iState;        // Integrator state
        float iMin, iMax;    // Maximum and minimum allowable integrator state
        float pGain,      // proportional gain
              iGain,      // integral gain
              dGain;      // derivative gain
    public:
        DigitalPID();
        float CalculatePID(float input);
        void setSetPoint(float newSetPoint);
};

// Constructor
DigitalPID::DigitalPID()
: iMin(-ITERM_MAX), iMax(ITERM_MAX), pGain(StabilP), iGain(StabilI), dGain(StabilD)
{
}

float DigitalPID::CalculatePID(float input)
{
    float error, output;
    error = setPoint - input;
    
    float pTerm, dTerm, iTerm;
    pTerm = pGain * error;   
    
    // calculate the integral state with appropriate limiting
    this->iState += constrain(error, -MAX_ERROR, MAX_ERROR);
    this->iState = constrain(this->iState, iMin, iMax);
    iTerm = this->iGain * iState * dt;  // calculate the integral term
  
    dTerm = this->dGain * (error - errorOld);
    errorOld = error;

    // Calkowanie metoda prostokata. Mozna trapez, mozna usprawnic rozniczkowanie
    // output = Kp*error - Td*(error - errorOld) + Ti*PID_errorSum*dt; ///PDTimer.read();
    // TODO: moze -Ti ?
    
    output = pTerm + iTerm - dTerm;
    // PWM Range
    output = constrain(output, -1.0, 1.0);
    return output;
}

void DigitalPID::setSetPoint(float newSetPoint)
{
    setPoint = newSetPoint;
}

// Old PID function
float StabilisationPD(float angle, float setPoint, float Kp, float Kd)
{/*
    float error;
    float output;

    error = angle-setPoint;
    output = Kp*error - Kd*(error - errorOld); // /PDTimer.read();
    
    // PWM Range
    output = constrain(output, -1.0, 1.0); // * -1
    
    errorOld = error;
    return(output);
*/   
    return -1; 
}

float constrain(float input, float min, float max)
{
    if(input < min)
        input = min;
    else if(input > max)
        input = max;
    return input;
}

/*
float StabilisationPID(float error, float setPoint, float Kp, float Ti, float Td)
{
    float output;
    float dt = StabilRate/1000000;

    PID_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
    PID_errorSum = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);

    // Calkowanie metoda prostokata. Mozna trapez, mozna usprawnic rozniczkowanie
    output = Kp*error - Td*(error - errorOld) + Ti*PID_errorSum*dt; ///PDTimer.read();
    // TODO: moze -Ti ?
    
    // PWM Range
    output = constrain(output, -1.0, 1.0);
    
    //PDTimer.reset();
    errorOld2 = errorOld;
    errorOld = error;
    
    return output;
}
*/

#endif // _DIGITALPID_H_
