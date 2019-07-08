#include <Servo.h>

//----------------------------------------------------------
// Potentiometer stuff
//----------------------------------------------------------
#define Pmeter A0
#define BL 13

//----------------------------------------------------------
// Serial Params
//----------------------------------------------------------
#define SHOW_DEBUG 0
#define FRAME_START '!'
#define FRAME_END '#'

//----------------------------------------------------------
// General Constants
//----------------------------------------------------------
#define SERVO_COUNT 6
#define FLOAT_BYTE_SIZE 4
#define ANGLE_MIN 50
#define ANGLE_MAX 130
#define D2R PI/180
#define R2D 180/PI

//----------------------------------------------------------
// Enums / States
//----------------------------------------------------------
enum OperationStates { Input_8Bit = 0, Input_Float32, CalibrateTrim };
OperationStates operationState = Input_Float32;

enum ReadStates { WaitingToStart, Reading, WaitingToEnd };
ReadStates currentReadState = WaitingToStart;

//----------------------------------------------------------
// Custom Data Types
//----------------------------------------------------------
struct Vector3
{
  float x;
  float y;
  float z;

  Vector3(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f)
  {
    x = _x;
    y = _y;
    z = _z;
  }

  // *** 4.) Student must fill the following operator overloads
  Vector3 operator+(const Vector3& a) const
  {
    return Vector3(x + a.x, y + a.y, z + a.z);
  }
  Vector3 operator-(const Vector3& a) const
  {
    return Vector3(x - a.x, y - a.y, z - a.z);
  }
  Vector3 operator*(const float& a) const
  {
    return Vector3(a * x, a * y, a * z);
  }
  Vector3 operator+=(const Vector3& a)
  {
    x = x + a.x;
    y = y + a.y;
    z = z + a.z;
    return Vector3(x, y, z);
  }
};

typedef union
{
  float floatingPoint;
  byte binary[FLOAT_BYTE_SIZE];
} BinaryFloat;

//----------------------------------------------------------
// Serial Comm Vars
//----------------------------------------------------------
byte inputBytes[SERVO_COUNT];
BinaryFloat inputFloats[SERVO_COUNT];
int byteIndex = 0;
int inputIndex = 0;

//----------------------------------------------------------
// Servo
//----------------------------------------------------------
Servo myServos[SERVO_COUNT];

// DOFs - This array holds the raw values for the desired position and orientation of the platform
// [0 = Sway (mm), 1 = Surge (mm), 2 = Heave (mm), 3 = Pitch (rads), 4 = Roll (rads), 5 = Yaw( rads)]
float DOFs[SERVO_COUNT] = { 0, 0, 0, 0, 0, 0 };

//----------------------------------------------------
// Modifiable Values
//----------------------------------------------------
int servoPins[] = { 3, 5, 6, 9, 10, 11 }; // servos must increment clockwise in order
int servoTrims[] = { -4, 8, 8, -8, 6, 6 }; // fine tuning adjustments
float baseRadius = 63.196f; // Base Radius
float platformRadius = 51.926f; // Platform Radius
float hornLength = 12.09f; // Servo Horn Length
float rodLength = 92.46f; // Connecting Arm Length
float baseAngleOffset = 17.511f; // The angle between the attach points on the base
float platformAngleOffset = 10.856f; // The angle between the attach points on the platform

//----------------------------------------------------
// Calculated Values
//----------------------------------------------------
float servoAxisAngles[SERVO_COUNT] = { 240, 60, 0, 180, 120, 300 }; //The angle that corresponds to each side (6 sides)
float platformDefaultHeight;
Vector3 platformDefaultHeightVector;

//----------------------------------------------------
// Dynamic (Allocate memory only when needed)
//----------------------------------------------------
Vector3* basePoints = new Vector3[6];
Vector3* platformPointOrigins = new Vector3[6];

//----------------------------------------------------
// Forward Declarations (due to lack of header file)
//----------------------------------------------------
void ChangeState(ReadStates _state);
void UpdateValues(bool showValues);
void SetServos(bool showValues);
void GetInputBytes();
void GetInputFloats();
void LogVector(String s, Vector3 v);
float GetMagnitude(Vector3 v);
float GetMagnitudeSquared(Vector3 v);
int Mod(int x, int m);
Vector3 CrossProduct(Vector3 v1, Vector3 v2);

void InitializePlatform();
void DefineBasePoints();
void DefinePlatformPointOrigins();
void CalculateDefaultHeight();

Vector3 DoRotate(int i);
Vector3 DoTranslate(int i);


int readings;

//----------------------------------------------------
// Setup
//----------------------------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(20);

//attach potentiometer
pinMode(BL,OUTPUT);

  // attach servos
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    myServos[i].attach(servoPins[i]);
  }

  delay(200);

  if (operationState == Input_Float32)
  {
    Serial.println("Using 6 DOF 32 Float mode");
  }
  else if (operationState == Input_8Bit)
  {
    Serial.println("Using 6 DOF Single Byte mode");
  }
  else if (operationState == CalibrateTrim)
  {
    Serial.println("Using CalibrateTrim mode");
    // Setting Trim
    for (int i = 0; i < SERVO_COUNT; i++)
    {
      // First, set the servos to an arbitrary value with a delay in between each one.
      // This allows us to visually confirm the Servo pin order
      myServos[i].write(90 + servoTrims[i] + (i % 2 == 0 ? 1 : -1) * 20);
      delay(200);
    }
    delay(400);
    for (int i = 0; i < SERVO_COUNT; i++)
    {
      // Now set the actual trim values and confirm that they are correct
      myServos[i].write(90 + servoTrims[i]);
      delay(200);
    }
    return;
  }

  InitializePlatform();
}

//----------------------------------------------------
// Initialize Platform
//----------------------------------------------------
void InitializePlatform()
{
  DefineBasePoints();
  DefinePlatformPointOrigins();
  CalculateDefaultHeight();

  if (SHOW_DEBUG) Serial.print("Default Height:"); Serial.println(platformDefaultHeight);

  // Set the height (z) value for each of our platformPointOrigins
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    platformPointOrigins[i].z = platformDefaultHeight;
  }

  ChangeState(ReadStates::WaitingToStart);

  UpdateValues(true);
}

//----------------------------------------------------
// Define Base Points
//----------------------------------------------------
void DefineBasePoints()
{
  // *** 1.) Student must fill this function to set the basePoints values for each of the 6 attachment points (Vector3[])
  // For example: basePoints[0] = { 5, 0, 0 };
  basePoints[0] = {-33.37f, 47.65f, 0};
  basePoints[1] = {-57.90f, 5.18f, 0};
  basePoints[2] = {-24.46f, -52.73f, 0};
  basePoints[3] = { 24.46f, -52.73f, 0};
  basePoints[4] = { 57.90f, 5.18f, 0};
  basePoints[5] = { 33.37f, 47.65f, 0};

  if (SHOW_DEBUG) for (int i = 0; i < SERVO_COUNT; i++) LogVector("basePoints " + i, basePoints[i]);
}

//----------------------------------------------------
// Define Platform Points
//----------------------------------------------------
void DefinePlatformPointOrigins()
{
  // *** 2.) Student must fill this function to set the basePoints values for each of the 6 attachment points (Vector3[])
  // For example: basePoints[0] = { 5, 0, 0 };
  platformPointOrigins[0] = {-6.69f, 46.11f, 0};
  platformPointOrigins[1] = {-43.24f, -17.35f, 0};
  platformPointOrigins[2] = {-36.54f, -28.91f, 0};
  platformPointOrigins[3] = { 36.54f, -28.91f, 0};
  platformPointOrigins[4] = { 43.24f, -17.35f, 0};
  platformPointOrigins[5] = { 6.69f, 46.11f, 0};

  if (SHOW_DEBUG) for (int i = 0; i < SERVO_COUNT; i++) LogVector("platformPointOrigins " + i, platformPointOrigins[i]);
}

//----------------------------------------------------
// Calcualte Default Height
//----------------------------------------------------
void CalculateDefaultHeight()
{
  // First we find the end point of the horn when it is flat (90 degrees). This is the point where it attaches
  // to the end of the rod via a ball joint. We'll use this point to determine the default height of the platform.
  // We only need to do this on one of the 6 arms, because the defining values are the same for all of them.
  // Note: In the calculation below, the + 90 degrees in the sin/cos is because we are defining our platform as
  // though it were rotated by 90 degrees along the up axis.
  float platformRotation = 90.0f;
  int armIndex = 0;
  Vector3 servoForwardAxis = Vector3(cos(D2R * (servoAxisAngles[armIndex] + platformRotation)), sin(D2R * (servoAxisAngles[armIndex] + platformRotation)), 0);

  // The CrossProduct is used to determine the direction vector of the servo horn when it's flat (90 degrees).
  // For example: hornDirection = Cross(servoForward, up)
  // *** 3.) Student must calulate the horn direction using the Cross Product
  Vector3 hornDirection = CrossProduct(servoForwardAxis, Vector3(0, 0, 1));
  Vector3 hornEndPoint = basePoints[armIndex] + (hornDirection * hornLength);

  if (SHOW_DEBUG) LogVector("hornEndPoint", hornEndPoint);

  // Calculate default height (Pythagorean theorem)
  float sideC = rodLength;
  float sideB = GetMagnitudeSquared(platformPointOrigins[armIndex] - hornEndPoint);
  float sideA = sqrt(sideC * sideC - sideB);  // sideB is already the squared length from the previous caluculation

  platformDefaultHeight = sideA;
  platformDefaultHeightVector = Vector3(0, 0, platformDefaultHeight);

  Serial.print("Default Height:"); Serial.println(platformDefaultHeight);
}

//----------------------------------------------------
// Main Loop
//----------------------------------------------------
void loop()
{
  if (operationState == Input_Float32)
  {
    GetInputFloats();
  }
  else if (operationState == Input_8Bit)
  {
    GetInputBytes();
  }

  //Serial.println(600);
  
  
}

//----------------------------------------------------
// Apply the transformations to get the newly positioned
// and rotated points for each platform attachment point.
//----------------------------------------------------
Vector3 DoRotate(int i)
{
  float x = platformPointOrigins[i].x * cos(Roll()) * cos(Yaw()) + platformPointOrigins[i].y * (sin(Pitch()) * sin(Roll()) * cos(Roll()) - cos(Pitch()) * sin(Yaw()));
  float y = platformPointOrigins[i].x * cos(Roll()) * sin(Yaw()) + platformPointOrigins[i].y * (cos(Pitch()) * cos(Yaw()) + sin(Pitch()) * sin(Roll()) * sin(Yaw()));
  float z = -platformPointOrigins[i].x * sin(Roll()) + platformPointOrigins[i].y * sin(Pitch()) * cos(Roll());
  return Vector3(x, y, z);
}

Vector3 DoTranslate(int i)
{
  // *** 3.) Student must fill this function to set the basePoints values for each of the 6 attachment points (Vector3[])
  // For example: return Vector3(5, 0, 0); // This would tell the platform to move to 5mm on the x axis (i.e. 5mm of Sway)
  return Vector3(Sway(), Surge(), Heave());
}

//----------------------------------------------------
// Write to our servos
//----------------------------------------------------
void UpdateValues(bool showValues)
{
  float servoAngleInRadians[] = { 0, 0, 0, 0, 0, 0 };
  Vector3 platformPoint = Vector3(0, 0, 0);

  for (int i = 0; i < SERVO_COUNT; i++)
  {
    // Perform rotation and translation operations on desired the DOF inputs (sway, surge, heave, pitch, roll, yaw)
    platformPoint = DoRotate(i) + DoTranslate(i) + platformDefaultHeightVector;

    // Virtual Rod Direction (Vector)
    Vector3 calculatedRodVector = basePoints[i] - platformPoint;
    if (SHOW_DEBUG) LogVector("calculatedRodVector", calculatedRodVector);

    // Virtual Rod Length (Magnitude)
    float calculatedRodLength = GetMagnitude(calculatedRodVector);
    if (SHOW_DEBUG) {
      Serial.print("calculatedRodLength:");
      Serial.println(calculatedRodLength);
    }

    // The difference (v1 - v2) of the vectors for each attachment point
    Vector3 platformDifference = platformPoint - basePoints[i];
    if (SHOW_DEBUG) LogVector("platformDifference", platformDifference);

    // Project the end point of the new virtual arm onto the servo's plane of rotation
    float L = sq(calculatedRodLength) - sq(rodLength) + sq(hornLength);
    if (SHOW_DEBUG) {
      Serial.print("L:");
      Serial.println(L);
    }

    float M = 2.0f * hornLength * platformDifference.z;
    if (SHOW_DEBUG) {
      Serial.print("M:");
      Serial.println(M);
    }

    float N = 2.0f * hornLength * (cos(D2R * servoAxisAngles[i]) * platformDifference.x + sin(D2R * servoAxisAngles[i]) * platformDifference.y);
    if (SHOW_DEBUG) {
      Serial.print("N:");
      Serial.println(N);
    }

    // Derive the servo angle (in radians) using inverse trig functions
    servoAngleInRadians[i] = asin(L / sqrt(M * M + N * N)) - atan(N / M);
    if (SHOW_DEBUG) {
      Serial.print("Radians:");
      Serial.println(servoAngleInRadians[i]);
    }
  }

  //----------------------------------------------------------------------------------
  // Add trim, check for error (NAN) values, and write to servos
  //----------------------------------------------------------------------------------
  float servoAngle = 0;
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    // Our calculations are worked out with zero degrees as the home angle,
    // so we need to add 90 degrees to account for the servo's home "flat" position
    servoAngle = (i % 2 == 0 ? -1 : 1) * (R2D * servoAngleInRadians[i]) + 90;

    // Error check, add Trim, and Constrain
    if (!isnan(servoAngle))
    {
      if (SHOW_DEBUG || showValues) {
        Serial.print("Servo Angle Before Constrain & Trim:");
        Serial.println(servoAngle);
      }
      servoAngle = constrain(servoAngle + servoTrims[i], ANGLE_MIN, ANGLE_MAX);
      myServos[i].write(servoAngle);
    }
    else if (SHOW_DEBUG) Serial.println("Error: Invalid Servo Angle");
  }
}

//----------------------------------------------------
// Getters - Encapsulate our DOF values for convenience
//----------------------------------------------------
float Sway() {
  return -DOFs[0];
}
float Surge() {
  return DOFs[1];
}
float Heave() {
  return DOFs[2];
}
float Pitch() {
  return D2R * -DOFs[3];
}
float Roll() {
  return D2R * DOFs[4];
}
float Yaw() {
  return D2R * DOFs[5];
}

//----------------------------------------------------
// Change State
//----------------------------------------------------
void ChangeState(ReadStates _state)
{
  currentReadState = _state;
}

//----------------------------------------------------
// Math Stuffs
//----------------------------------------------------
Vector3 CrossProduct(Vector3 v1, Vector3 v2)
{
  double x, y, z;
  x = v1.y * v2.z - v2.y * v1.z;
  y = (v1.x * v2.z - v2.x * v1.z) * -1;
  z = v1.x * v2.y - v2.x * v1.y;
  return Vector3(x, y, z);
}

float GetMagnitude(Vector3 v)
{
  // *** 5.) Student must fill in this function to return the magnitude of the Vector3 passed in
  return sqrt(sq(v.x) + sq(v.y) + sq(v.z));
}
float GetMagnitudeSquared(Vector3 v)
{
  // *** 6.) Student must fill in this function to return the squared magnitude of the Vector3 passed in
  return (sq(v.x) + sq(v.y) + sq(v.z));
}

int Mod(int x, int m) {
  return (x % m + m) % m;
}

float MapRange(float val, float min, float max, float newMin, float newMax)
{
  return ((val - min) / (max - min) * (newMax - newMin) + newMin);
}

void LogVector(String name, Vector3 v)
{
  Serial.print('\\' + name + ": ( ");
  Serial.print(v.x);
  Serial.print(", ");
  Serial.print(v.y);
  Serial.print(", ");
  Serial.print(v.z);
  Serial.println(")");
}

//----------------------------------------------------
// Read and Parse Serial Input
//----------------------------------------------------
void GetInputFloats()
{
  while (Serial.available() > 0)
  {
    byte inByte = Serial.read(); // read the next byte in the stream

    //----------------------------------------------------
    // Waiting for start
    //----------------------------------------------------
    if (currentReadState == WaitingToStart)
    {
      if ((char)inByte == FRAME_START)
      {
        ChangeState(Reading);
        inputIndex = 0;
      }
    }
    //----------------------------------------------------
    // Reading - message is being read
    //----------------------------------------------------
    else if (currentReadState == Reading)
    {
      inputFloats[inputIndex].binary[byteIndex] = inByte;
      if (byteIndex < FLOAT_BYTE_SIZE - 1)
      {
        byteIndex++;
      }
      else
      {
        byteIndex = 0;
        DOFs[inputIndex] = inputFloats[inputIndex].floatingPoint;

        // Have we read values for all servos?
        if (inputIndex < SERVO_COUNT - 1)
        {
          inputIndex++;
        }
        else
        {
          ChangeState(WaitingToEnd);
        }
      }
    }
    //----------------------------------------------------
    // Waiting for end
    //----------------------------------------------------
    else if (currentReadState == WaitingToEnd)
    {
      ChangeState(ReadStates::WaitingToStart);
      if ((char)inByte == FRAME_END)
      {
        if (SHOW_DEBUG) Serial.println("---> Message Received");
        UpdateValues(false);
      }
      else
      {
        if (SHOW_DEBUG) Serial.println("---> Message Discarded");
      }
    }
  }
}

void GetInputBytes()
{
  while (Serial.available() > 0)
  {
    byte inByte = Serial.read(); // read the next byte in the stream

    //----------------------------------------------------
    // Waiting for start
    //----------------------------------------------------
    if (currentReadState == WaitingToStart)
    {
      if ((char)inByte == FRAME_START)
      {
        ChangeState(Reading);
        inputIndex = 0;
      }
    }
    //----------------------------------------------------
    // Reading - message is being read
    //----------------------------------------------------
    else if (currentReadState == Reading)
    {
      inputBytes[inputIndex] = (int)inByte;

      DOFs[inputIndex] = MapRange(inputBytes[inputIndex], 0.0f, 255.0f, -15.0f, 15.0f);

      // Have we read values for all servos?
      if (inputIndex < SERVO_COUNT - 1)
      {
        inputIndex++;
      }
      else
      {
        ChangeState(WaitingToEnd);
      }
    }
    //----------------------------------------------------
    // Waiting for end
    //----------------------------------------------------
    else if (currentReadState == WaitingToEnd)
    {
      ChangeState(WaitingToStart);
      if ((char)inByte == FRAME_END)
      {
        if (SHOW_DEBUG) Serial.println("---> Message Received");
        UpdateValues(false);
      }
      else
      {
        if (SHOW_DEBUG) Serial.println("---> Message Discarded");
      }
    }
  }
}
