#include <Servo.h>
#include <HandJoint.h>c:\Users\Uporabnik\Documents\Arduino\libraries\HandJoint\HandConstants.h
#include <HandConstants.h>

// Instruction variables
String spFIFO;
int sizeFIFO;
bool state;c:\Users\Uporabnik\Documents\Arduino\libraries\HandJoint\HandJoint.cpp
unsigned long lastSend, tempTime; 

HandJoint MCPR_L(522, 662, 0);
HandJoint MCPR_R(488, 654, 0);
HandJoint MCPR_M(505, 646, 0);
HandJoint MCPR_I(532, 696, 0);
HandJoint MCPR_T(472, 670, 0);

Servo MCPR_L_servo;
Servo MCPR_R_servo;
Servo MCPR_M_servo;
Servo MCPR_I_servo;
Servo MCPR_T_servo;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    // set state to pause
    state = false;
    sizeFIFO = 0;

    // Connecting servos to PWM pins and set min and max values
    MCPR_L_servo.attach(MCPR_L_SERVO, 500, 2500);
    MCPR_R_servo.attach(MCPR_R_SERVO, 500, 2500);
    MCPR_M_servo.attach(MCPR_M_SERVO, 500, 2500);
    MCPR_I_servo.attach(MCPR_I_SERVO, 500, 2500);
    MCPR_T_servo.attach(MCPR_T_SERVO, 500, 2500);

    // Read initial joint positions and init filtering arrays
    MCPR_L.initFiltering(analogRead(MCPR_L_POTENTIOMETER));
    MCPR_R.initFiltering(analogRead(MCPR_R_POTENTIOMETER));
    MCPR_M.initFiltering(analogRead(MCPR_M_POTENTIOMETER));
    MCPR_I.initFiltering(analogRead(MCPR_I_POTENTIOMETER));
    MCPR_T.initFiltering(analogRead(MCPR_T_POTENTIOMETER));

    // Initial joint positions are whatever the current position is, because we don't want anything to move on startup
    MCPR_L.updateSetpoint(MCPR_L.joint_position);
    MCPR_R.updateSetpoint(MCPR_R.joint_position);
    MCPR_M.updateSetpoint(MCPR_M.joint_position);
    MCPR_I.updateSetpoint(MCPR_I.joint_position);
    MCPR_T.updateSetpoint(MCPR_T.joint_position); 

    // potentially need to invert all or none - NEEDS CHECKING
    
    // Don't allow motor movement initially
    stopAllMotors();
}

void loop() {
    while (true) {
        //runTime = micros();

        // Update potentiometer values
        MCPR_L.updateJointPosition(analogRead(MCPR_L_POTENTIOMETER));
        MCPR_R.updateJointPosition(analogRead(MCPR_R_POTENTIOMETER));
        MCPR_M.updateJointPosition(analogRead(MCPR_M_POTENTIOMETER));
        MCPR_I.updateJointPosition(analogRead(MCPR_I_POTENTIOMETER));
        MCPR_T.updateJointPosition(analogRead(MCPR_T_POTENTIOMETER));

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

    MCPR_L_servo.writeMicroseconds(MCPR_L.regulateJoint(1, 1));
    MCPR_R_servo.writeMicroseconds(MCPR_R.regulateJoint(1, 1));
    MCPR_M_servo.writeMicroseconds(MCPR_M.regulateJoint(1, 1));
    MCPR_I_servo.writeMicroseconds(MCPR_I.regulateJoint(1, 1));
    MCPR_T_servo.writeMicroseconds(MCPR_T.regulateJoint(1, 1));

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
    /*
    Serial.println(sp_val);
    Serial.println(spFIFO.length());
    Serial.println(spFIFO);*/
    
    switch (sp_id) {
        case MCPR_L_ID:
            MCPR_L.updateSetpoint(sp_val);
        case MCPR_R_ID:
            MCPR_R.updateSetpoint(sp_val);
        case MCPR_M_ID:
            MCPR_M.updateSetpoint(sp_val);        
        case MCPR_I_ID:
            MCPR_I.updateSetpoint(sp_val);
        case MCPR_T_ID:
            MCPR_T.updateSetpoint(sp_val);     
    }

    // delete the updated joint setpoint
    spFIFO = spFIFO.substring(4, -1);
    sizeFIFO -= 1;
}


void stopAllMotors() {
    MCPR_L_servo.writeMicroseconds(1500);
    MCPR_R_servo.writeMicroseconds(1500);
    MCPR_M_servo.writeMicroseconds(1500);
    MCPR_I_servo.writeMicroseconds(1500);
    MCPR_T_servo.writeMicroseconds(1500);
}

void plot() {
    Serial.print("static1:");
    Serial.print(650);
    Serial.print("\t");
    Serial.print("static2:");
    Serial.print(450);
    Serial.print("\t");
    Serial.print("Little MCPR:");
    Serial.print(MCPR_L.joint_position);
    Serial.print("\t");
    Serial.print("Ring MCPR:");
    Serial.print(MCPR_R.joint_position);
    Serial.print("\t");
    Serial.print("Middle MCPR:");
    Serial.print(MCPR_M.joint_position);
    Serial.print("\t");
    Serial.print("Index MCPR:");
    Serial.print(MCPR_I.joint_position);
    Serial.print("\t");
    Serial.print("Thumb MCPR:");
    Serial.print(MCPR_T.joint_position);
    Serial.println();
}

void sendStateData() {
    // format: {{state:1},{A:999,B:999,C:999,D:999,Z:999}}
    Serial.print("{{id:2},{state:");
    Serial.print(state);
    Serial.print("},{D:");
    Serial.print(MCPR_L.joint_position);
    Serial.print(",H:");
    Serial.print(MCPR_R.joint_position);
    Serial.print(",L:");
    Serial.print(MCPR_M.joint_position);
    Serial.print(",P:");
    Serial.print(MCPR_I.joint_position);
    Serial.print(",Q:");
    Serial.print(MCPR_T.joint_position);
    Serial.print("}}\n");
}
