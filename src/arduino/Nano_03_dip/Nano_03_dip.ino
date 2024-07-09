#include <Servo.h>
#include <HandJoint.h>
#include <HandConstants.h>

// Communication variables
String spFIFO;
int sizeFIFO;
bool state;
unsigned long lastSend, tempTime; // runTime

HandJoint MCP_M(478, 897, 1);
HandJoint PIP_M(198, 967, 2);
HandJoint DIP_M(210, 951, 3);
HandJoint MCP_I(487, 902, 1);
HandJoint PIP_I(172, 941, 2);
HandJoint DIP_I(243, 900, 3);

Servo MCP_M_servo;
Servo PIP_M_servo;
Servo DIP_M_servo;
Servo MCP_I_servo;
Servo PIP_I_servo;
Servo DIP_I_servo;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    // Set state to pause
    state = false;
    sizeFIFO = 0;

    // Connecting servos to PWM pins and set min and max values
    MCP_M_servo.attach(MCP_M_SERVO, 500, 2500);
    PIP_M_servo.attach(PIP_M_SERVO, 500, 2500);
    DIP_M_servo.attach(DIP_M_SERVO, 500, 2500);
    MCP_I_servo.attach(MCP_I_SERVO, 500, 2500);
    PIP_I_servo.attach(PIP_I_SERVO, 500, 2500);
    DIP_I_servo.attach(DIP_I_SERVO, 500, 2500);

    // Read initial joint positions and init filtering arrays
    MCP_M.initFiltering(analogRead(MCP_M_POTENTIOMETER));
    PIP_M.initFiltering(analogRead(PIP_M_POTENTIOMETER));
    DIP_M.initFiltering(analogRead(DIP_M_POTENTIOMETER));
    MCP_I.initFiltering(analogRead(MCP_I_POTENTIOMETER));
    PIP_I.initFiltering(analogRead(PIP_I_POTENTIOMETER));
    DIP_I.initFiltering(analogRead(DIP_I_POTENTIOMETER));
    
    // Initial joint positions are whatever the current position is, because we don't want anything to move on startup
    MCP_M.updateSetpoint(MCP_M.joint_position);
    PIP_M.updateSetpoint(PIP_M.joint_position);
    DIP_M.updateSetpoint(DIP_M.joint_position);
    MCP_I.updateSetpoint(MCP_I.joint_position);
    PIP_I.updateSetpoint(PIP_I.joint_position);
    DIP_I.updateSetpoint(DIP_I.joint_position);

    // Invert servo rotation on PIP and DIP joints, because the strings are twisted
    PIP_M.invertServoRotation();
    DIP_M.invertServoRotation();
    PIP_I.invertServoRotation();
    DIP_I.invertServoRotation();

    // Don't allow motor movement initially
    stopAllMotors();
}

void loop() {
    while (true) {
        //runTime = micros();

        // Update potentiometer values
        MCP_M.updateJointPosition(analogRead(MCP_M_POTENTIOMETER));
        PIP_M.updateJointPosition(analogRead(PIP_M_POTENTIOMETER));
        DIP_M.updateJointPosition(analogRead(DIP_M_POTENTIOMETER));
        MCP_I.updateJointPosition(analogRead(MCP_I_POTENTIOMETER));
        PIP_I.updateJointPosition(analogRead(PIP_I_POTENTIOMETER));
        DIP_I.updateJointPosition(analogRead(DIP_I_POTENTIOMETER));

        // Periodicaly send data, limited to set period to not spam serial connection which can cause problems
        tempTime = micros();
        if (tempTime - lastSend > 500000) {
            sendStateData();
            //plot();
            lastSend = tempTime;
        }

        // Check for new instructions
        checkForInstructions();

        // If sp buffer isn't empty, update the next SP
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

    //tmp
    DIP_M.regulateJoint(1, 1);
    PIP_M.regulateJoint(1, DIP_M.joint_movement_available);
    DIP_I.regulateJoint(1, 1);
    PIP_I.regulateJoint(1, DIP_I.joint_movement_available);
    
    // Regulate joints based on the error in positions
    //DIP_M_servo.writeMicroseconds(DIP_M.regulateJoint(1, 1));
    //PIP_M_servo.writeMicroseconds(PIP_M.regulateJoint(1, DIP_M.joint_movement_available)); 
    MCP_M_servo.writeMicroseconds(MCP_M.regulateJoint(PIP_M.joint_movement_available, DIP_M.joint_movement_available));

    //DIP_I_servo.writeMicroseconds(DIP_I.regulateJoint(1, 1));
    //PIP_I_servo.writeMicroseconds(PIP_I.regulateJoint(1, DIP_I.joint_movement_available));
    MCP_I_servo.writeMicroseconds(MCP_I.regulateJoint(PIP_I.joint_movement_available, DIP_I.joint_movement_available));
    
    //while (micros() - runTime < 100000) {}
}

// check serial connection for new commands, current available commands are W, A, S;
void checkForInstructions() {
    String input;

    if (Serial.available()) {
        input = Serial.readString();
        switch (input[0]) {
            case 'W': // update set-point values
                sizeFIFO += (int)(input[1] - 48); // number 0 has ASCII code 48
                // second index until the end of msg, 
                // can't put -1 because serial adds a sign at the end
                spFIFO += input.substring(2, (int)(input[1] - 48) * 4 + 2); 
                break;
            case 'A': // change state to active
                state = true;
                break;
            case 'S': // change state to stop
                state = false;
                break;
            // potentially add additional commands
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
        case MCP_M_ID:
            MCP_M.updateSetpoint(sp_val);
        case PIP_M_ID:
            PIP_M.updateSetpoint(sp_val);
        case DIP_M_ID:
            DIP_M.updateSetpoint(sp_val);        
        case MCP_I_ID:
            MCP_I.updateSetpoint(sp_val);
        case PIP_I_ID:
            PIP_I.updateSetpoint(sp_val);
        case DIP_I_ID:
            DIP_I.updateSetpoint(sp_val);      
    }

    // delete the updated joint setpoint
    spFIFO = spFIFO.substring(4, -1);
    sizeFIFO -= 1;
}

void stopAllMotors() {
    MCP_M_servo.writeMicroseconds(1500);
    PIP_M_servo.writeMicroseconds(1500);
    DIP_M_servo.writeMicroseconds(1500);
    MCP_I_servo.writeMicroseconds(1500);
    PIP_I_servo.writeMicroseconds(1500);
    DIP_I_servo.writeMicroseconds(1500);
}

void plot() {
    Serial.print("static1:");
    Serial.print(650);
    Serial.print("\t");
    Serial.print("static2:");
    Serial.print(450);
    Serial.print("\t");
    Serial.print("Middle MCP:");
    Serial.print(MCP_M.joint_position);
    Serial.print("\t");
    Serial.print("Middle PIP:");
    Serial.print(PIP_M.joint_position);
    Serial.print("\t");
    Serial.print("Middle DIP:");
    Serial.print(DIP_M.joint_position);
    Serial.print("\t");
    Serial.print("Index MCP:");
    Serial.print(MCP_I.joint_position);
    Serial.print("\t");
    Serial.print("Index PIP:");
    Serial.print(PIP_I.joint_position);
    Serial.print("\t");
    Serial.print("Index DIP:");
    Serial.print(DIP_I.joint_position);
    Serial.println();
}

void sendStateData() {
    // format: {{state:1},{A:999,B:999,C:999,D:999,Z:999}}
    Serial.print("{{id:3},{state:");
    Serial.print(state);
    Serial.print("},{I:");
    Serial.print(MCP_M.joint_position);
    Serial.print(",J:");
    Serial.print(DIP_M.joint_position);
    Serial.print(",K:");
    Serial.print(PIP_M.joint_position);
    Serial.print(",M:");
    Serial.print(MCP_I.joint_position);
    Serial.print(",N:");
    Serial.print(DIP_I.joint_position);
    Serial.print(",O:");
    Serial.print(PIP_I.joint_position);
    Serial.print("}}\n");
}
