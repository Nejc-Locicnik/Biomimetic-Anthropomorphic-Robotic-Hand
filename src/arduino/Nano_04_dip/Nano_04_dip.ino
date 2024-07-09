#include <Servo.h>
#include <HandJoint.h>
#include <HandConstants.h>

// Instruction variables
String spFIFO;
int sizeFIFO;
bool state;
unsigned long lastSend, tempTime; // runTime

HandJoint MCP_T(237, 629, 1);
HandJoint PIP_T(198, 934, 2);
HandJoint DIP_T(172, 901, 3);

Servo MCP_T_servo;
Servo PIP_T_servo;
Servo DIP_T_servo;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    // set state to pause
    state = false;
    sizeFIFO = 0;

    // Connecting servos to PWM pins and set min and max values
    MCP_T_servo.attach(MCP_T_SERVO, 500, 2500);
    PIP_T_servo.attach(PIP_T_SERVO, 500, 2500);
    DIP_T_servo.attach(DIP_T_SERVO, 500, 2500);

    // Read initial joint positions and init filtering arrays
    MCP_T.initFiltering(analogRead(MCP_T_POTENTIOMETER));
    PIP_T.initFiltering(analogRead(PIP_T_POTENTIOMETER));
    DIP_T.initFiltering(analogRead(DIP_T_POTENTIOMETER));

    // Initial joint positions are whatever the current position is, because we don't want anything to move on startup
    MCP_T.updateSetpoint(MCP_T.joint_position);
    PIP_T.updateSetpoint(PIP_T.joint_position);
    DIP_T.updateSetpoint(DIP_T.joint_position);

    // Invert servo rotation on PIP and DIP joints, because the strings are twisted
    PIP_T.invertServoRotation();
    DIP_T.invertServoRotation();
    
    // Don't allow motor movement initially
    stopAllMotors();
}

void loop() {
    while (true) {
        //runTime = micros();

        // Update potentiometer values
        MCP_T.updateJointPosition(analogRead(MCP_T_POTENTIOMETER));
        PIP_T.updateJointPosition(analogRead(PIP_T_POTENTIOMETER));
        DIP_T.updateJointPosition(analogRead(DIP_T_POTENTIOMETER));

        // Periodicaly send data, limited to set period to not spam serial connection which can cause problems
        tempTime = micros();
        if (tempTime - lastSend > 500000) {
            sendStateData();
            //plot();
            lastSend = tempTime;
        }

        // Check for new instructions
        checkForInstructions();

        // If sp buffer isnt empty, update the next SP
        if (sizeFIFO > 0) {
            updateJointSP();
        }

        // Break the while loop if state is Active, else if state is Stop, stop all motors
        if (state) {
            break;          
        } else {
            stopAllMotors();
        }
    }

    // Regulate joints based on the error in positions
    DIP_T_servo.writeMicroseconds(DIP_T.regulateJoint(1, 1));
    PIP_T_servo.writeMicroseconds(PIP_T.regulateJoint(1, DIP_T.joint_movement_available));
    MCP_T_servo.writeMicroseconds(MCP_T.regulateJoint(PIP_T.joint_movement_available, DIP_T.joint_movement_available));

    //while (micros() - runTime < 100000) {}
}

// check serial connection for new commands, current available commands are W, A, S, C;
void checkForInstructions() {
    String input;

    if (Serial.available()) {
        input = Serial.readString();

        switch (input[0]) {
            case 'W': // update set-point values
                sizeFIFO += (int)(input[1] - 48); // number 0 has ASCII code 48, so we reduce the value by 48 to get proper integers
                spFIFO += input.substring(2, (int)(input[1] - 48) * 4 + 2); // second index until the end of msg, can't put -1 because serial adds a sign at the end
                break;
            case 'A': // change state to active
                state = true;
                break;
            case 'S': // change state to stop
                state = false;
                break;
            case 'C': // clear set-point FIFO
                sizeFIFO = 0;
                spFIFO = "";
                break;
            // potentially add additional commands, such as adjusting PID constants
        }
        //Serial.println("Recieved!");
    }
}

// update the first joint set-point (SP value) in the FIFO
void updateJointSP() {
    // update first sp in buffer before deleting it
    char sp_id = spFIFO[0];
    int sp_val = spFIFO.substring(1, 4).toInt();
    
    switch (sp_id) {
        case MCP_T_ID:
            MCP_T.updateSetpoint(sp_val);
        case PIP_T_ID:
            PIP_T.updateSetpoint(sp_val);
        case DIP_T_ID:
            DIP_T.updateSetpoint(sp_val);            
    }

    // delete the updated joint setpoint
    spFIFO = spFIFO.substring(4, -1);
    sizeFIFO -= 1;
}

void stopAllMotors() {
    MCP_T_servo.writeMicroseconds(1500);
    PIP_T_servo.writeMicroseconds(1500);
    DIP_T_servo.writeMicroseconds(1500);
}

void plot() {
    Serial.print("static1:");
    Serial.print(650);
    Serial.print("\t");
    Serial.print("static2:");
    Serial.print(450);
    Serial.print("\t");
    Serial.print("Thumb MCP:");
    Serial.print(MCP_T.joint_position);
    Serial.print("\t");
    Serial.print("Thumb PIP:");
    Serial.print(PIP_T.joint_position);
    Serial.print("\t");
    Serial.print("Thumb DIP:");
    Serial.print(DIP_T.joint_position);
    Serial.println();
}

void sendStateData() {
    // format: {{state:1},{A:999,B:999,C:999,D:999,Z:999}}
    Serial.print("{{id:4},{state:");
    Serial.print(state);
    Serial.print("},{R:");
    Serial.print(MCP_T.joint_position);
    Serial.print(",S:");
    Serial.print(DIP_T.joint_position);
    Serial.print(",T:");
    Serial.print(PIP_T.joint_position);
    Serial.print("}}\n");
}
