#include <Servo.h>
#include <HandJoint.h>
#include <HandConstants.h>

// Communication variables
String spFIFO;
int sizeFIFO;
bool state;
unsigned long lastSend, tempTime; // runTime

HandJoint MCP_L(500, 912, 1);
HandJoint PIP_L(207, 998, 2);
HandJoint DIP_L(234, 920, 3);
HandJoint MCP_R(470, 874, 1);
HandJoint PIP_R(189, 959, 2);
HandJoint DIP_R(223, 927, 3);

Servo MCP_L_servo;
Servo PIP_L_servo;
Servo DIP_L_servo;
Servo MCP_R_servo;
Servo PIP_R_servo;
Servo DIP_R_servo;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    // Set state to pause
    state = false;
    sizeFIFO = 0;
    
    // Connecting servos to PWM pins and set min and max values
    MCP_L_servo.attach(MCP_L_SERVO, 500, 2500);
    PIP_L_servo.attach(PIP_L_SERVO, 500, 2500);
    DIP_L_servo.attach(DIP_L_SERVO, 500, 2500);
    MCP_R_servo.attach(MCP_R_SERVO, 500, 2500);
    PIP_R_servo.attach(PIP_R_SERVO, 500, 2500);
    DIP_R_servo.attach(DIP_R_SERVO, 500, 2500);

    // Read initial joint positions and init filtering arrays
    MCP_L.initFiltering(analogRead(MCP_L_POTENTIOMETER));
    PIP_L.initFiltering(analogRead(PIP_L_POTENTIOMETER));
    DIP_L.initFiltering(analogRead(DIP_L_POTENTIOMETER));
    MCP_R.initFiltering(analogRead(MCP_R_POTENTIOMETER));
    PIP_R.initFiltering(analogRead(PIP_R_POTENTIOMETER));
    DIP_R.initFiltering(analogRead(DIP_R_POTENTIOMETER));

    // Initial joint positions are whatever the current position is, because we don't want anything to move on startup
    MCP_L.updateSetpoint(MCP_L.joint_position);
    PIP_L.updateSetpoint(PIP_L.joint_position);
    DIP_L.updateSetpoint(DIP_L.joint_position);
    MCP_R.updateSetpoint(MCP_R.joint_position);
    PIP_R.updateSetpoint(PIP_R.joint_position);
    DIP_R.updateSetpoint(DIP_R.joint_position); 

    // Invert servo rotation on PIP and DIP joints, because the strings are twisted
    PIP_L.invertServoRotation();
    DIP_L.invertServoRotation();
    PIP_R.invertServoRotation();
    DIP_R.invertServoRotation();

    // Don't allow motor movement initially
    stopAllMotors();
}

void loop() {
    while (true) {
        //runTime = micros();

        // Update potentiometer values
        MCP_L.updateJointPosition(analogRead(MCP_L_POTENTIOMETER));
        PIP_L.updateJointPosition(analogRead(PIP_L_POTENTIOMETER));
        DIP_L.updateJointPosition(analogRead(DIP_L_POTENTIOMETER));
        MCP_R.updateJointPosition(analogRead(MCP_R_POTENTIOMETER));
        PIP_R.updateJointPosition(analogRead(PIP_R_POTENTIOMETER));
        DIP_R.updateJointPosition(analogRead(DIP_R_POTENTIOMETER));

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

    // Calculate PID value and move motors
    //Serial.println(DIP_L.regulateJoint(1, 1));
    /*
    DIP_L.regulateJoint(1, 1);
    PIP_L.regulateJoint(1, DIP_L.joint_movement_available);
    DIP_R.regulateJoint(1, 1);
    PIP_R.regulateJoint(1, DIP_R.joint_movement_available);*/
    
    DIP_L_servo.writeMicroseconds(DIP_L.regulateJoint(1, 1));
    PIP_L_servo.writeMicroseconds(PIP_L.regulateJoint(1, DIP_L.joint_movement_available));
    MCP_L_servo.writeMicroseconds(MCP_L.regulateJoint(PIP_L.joint_movement_available, DIP_L.joint_movement_available));
    
    DIP_R_servo.writeMicroseconds(DIP_R.regulateJoint(1, 1));
    PIP_R_servo.writeMicroseconds(PIP_R.regulateJoint(1, DIP_R.joint_movement_available));
    MCP_R_servo.writeMicroseconds(MCP_R.regulateJoint(PIP_R.joint_movement_available, DIP_R.joint_movement_available));
    
    //while (micros() - runTime < 100000) {}
    ///delay(500);
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
            /*
            case 'C': // clear set-point FIFO
                sizeFIFO = 0;
                spFIFO = "";
                break;*/
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
        case MCP_L_ID:
            MCP_L.updateSetpoint(sp_val);
        case PIP_L_ID:
            PIP_L.updateSetpoint(sp_val);
        case DIP_L_ID:
            DIP_L.updateSetpoint(sp_val);        
        case MCP_R_ID:
            MCP_R.updateSetpoint(sp_val);
        case PIP_R_ID:
            PIP_R.updateSetpoint(sp_val);
        case DIP_R_ID:
            DIP_R.updateSetpoint(sp_val);      
    }

    // delete the updated joint setpoint
    spFIFO = spFIFO.substring(4, -1);
    sizeFIFO -= 1;
}

void stopAllMotors() {
    MCP_L_servo.writeMicroseconds(1500);
    PIP_L_servo.writeMicroseconds(1500);
    DIP_L_servo.writeMicroseconds(1500);
    MCP_R_servo.writeMicroseconds(1500);
    PIP_R_servo.writeMicroseconds(1500);
    DIP_R_servo.writeMicroseconds(1500);
}

void plot() {
    Serial.print("static1:");
    Serial.print(650);
    Serial.print("\t");
    Serial.print("static2:");
    Serial.print(450);
    Serial.print("\t");
    Serial.print("Little MCP:");
    Serial.print(MCP_L.joint_position);
    Serial.print("\t");
    Serial.print("Little PIP:");
    Serial.print(PIP_L.joint_position);
    Serial.print("\t");
    Serial.print("Little DIP:");
    Serial.print(DIP_L.joint_position);
    Serial.print("\t");
    Serial.print("Ring MCP:");
    Serial.print(MCP_R.joint_position);
    Serial.print("\t");
    Serial.print("Ring PIP:");
    Serial.print(PIP_R.joint_position);
    Serial.print("\t");
    Serial.print("Ring DIP:");
    Serial.print(DIP_R.joint_position);
    Serial.println();
}

void sendStateData() {
    // format: {{state:1},{A:999,B:999,C:999,D:999,Z:999}}
    Serial.print("{{id:1},{state:");
    Serial.print(state);
    Serial.print("},{A:");
    Serial.print(MCP_L.joint_position);
    Serial.print(",B:");
    Serial.print(DIP_L.joint_position);
    Serial.print(",C:");
    Serial.print(PIP_L.joint_position);
    Serial.print(",E:");
    Serial.print(MCP_R.joint_position);
    Serial.print(",F:");
    Serial.print(DIP_R.joint_position);
    Serial.print(",G:");
    Serial.print(PIP_R.joint_position);
    Serial.print("}}\n");
}
