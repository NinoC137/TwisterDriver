//
// Created by Yoshi on 2024/3/10.
//

#include "cmd_Parse.h"

void cmd_startParse(char* JsonString){
    cJSON *root = cJSON_Parse(JsonString);
    if(root == NULL){
        uart3_printf("Error before: [%s]\n", cJSON_GetErrorPtr());
    }
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    switch (cmd->valueint) {
        case 1:
            cmd_setAngularDeviation(root);
            break;
        case 2:
            cmd_setServoAngle(root);
            break;
        case 3:
            cmd_getMotorSpeed(root);
            break;
        case 4:
            cmd_getMotorAngle(root);
            break;
        case 5:
            cmd_getMotorCurrent(root);
            break;
        case 6:
            cmd_getSystemMode(root);
            break;
        case 7:
            cmd_setHeartBeat(root);
            break;
        default:
            break;
    }
    cJSON_Delete(root);
}

void res_sendHeartBeat(){

}

void cmd_setAngularDeviation(cJSON* root){

}

void cmd_setServoAngle(cJSON* root){

}

void cmd_getMotorSpeed(cJSON* root){

}

void cmd_getMotorAngle(cJSON* root){

}

void cmd_getMotorCurrent(cJSON* root){

}

void cmd_getSystemMode(cJSON* root){

}

void cmd_setHeartBeat(cJSON* root){

}