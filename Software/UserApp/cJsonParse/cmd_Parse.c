//
// Created by Yoshi on 2024/4/10.
//

#include "cmd_Parse.h"

void cmd_startParse(char* JsonString){
    cJSON *root = cJSON_Parse(JsonString);
    if(root == NULL){
        JSON_response("Error before: [%s]\n", cJSON_GetErrorPtr());
        return;
    }
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (!cJSON_IsNumber(cmd)) {
        JSON_response("Error: missing or invalid cmd\r\n");
        cJSON_Delete(root);
        return;
    }
    switch (cmd->valueint) {
        case 1:
            cmd_setAngularDeviation(root);
            break;
        case 2:
            cmd_setMotorAngle(root);
            break;
        case 3:
            cmd_setServoAngle(root);
            break;
        case 4:
            cmd_getMotorSpeed(root);
            break;
        case 5:
            cmd_getMotorAngle(root);
            break;
        case 6:
            cmd_getMotorCurrent(root);
            break;
        case 7:
            cmd_getSystemMode(root);
            break;
        case 8:
            cmd_setHeartBeat(root);
            break;
        case 9:
            cmd_setFOCMode(root);
            break;
        default:
            break;
    }
    cJSON_Delete(root);
}

void res_sendHeartBeat(){
    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "name", cJSON_CreateString(sysLog.name));
    cJSON_AddItemToObject(response_root, "lastUpgrade", cJSON_CreateString(sysLog.lastUpgradeTime));
    cJSON_AddItemToObject(response_root, "sysRunTime", cJSON_CreateNumber(sysLog.sysRunTime));
    cJSON_AddItemToObject(response_root, "beatTime_ms", cJSON_CreateNumber(sysLog.beatTime_ms));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_setAngularDeviation(cJSON* root){
    cJSON *cmd_Deviation = cJSON_GetObjectItem(root, "Deviation");
    if(cmd_Deviation == NULL){
        JSON_response("Error before: [%s]\n", cJSON_GetErrorPtr());
        return;
    }
    JSON_response("get Deviation: %d\r\n", cmd_Deviation->valueint);

    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(1));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}
extern volatile float targetMotorAngle_Left;
extern volatile float targetMotorAngle_Right;
extern volatile FOC_CONTROL_MODE focControlMode;
void cmd_setMotorAngle(cJSON *root){
    cJSON *cmd_AngleLeft = cJSON_GetObjectItem(root, "MotorAngle_Left");
    cJSON *cmd_AngleRight = cJSON_GetObjectItem(root, "MotorAngle_Right");
    if(!cJSON_IsNumber(cmd_AngleLeft) || !cJSON_IsNumber(cmd_AngleRight)){
        JSON_response("Error: missing or invalid motor angle\r\n");
        return;
    }

    targetMotorAngle_Left = (float)cmd_AngleLeft->valuedouble;
    targetMotorAngle_Right = (float)cmd_AngleRight->valuedouble;
    focControlMode = POSITION_CONTROL;

    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(2));
    cJSON_AddItemToObject(response_root, "NewMotorAngle_Left", cJSON_CreateNumber(targetMotorAngle_Left));
    cJSON_AddItemToObject(response_root, "NewMotorAngle_Right", cJSON_CreateNumber(targetMotorAngle_Right));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_setServoAngle(cJSON* root){
    cJSON *cmd_AngleLeft = cJSON_GetObjectItem(root, "ServoAngle_Left");
    cJSON *cmd_AngleRight = cJSON_GetObjectItem(root, "ServoAngle_Right");
    if(cmd_AngleLeft == NULL || cmd_AngleRight == NULL){
        JSON_response("Error before: [%s]\n", cJSON_GetErrorPtr());
        return;
    }

    setAngle_270(&Servo_LeftLeg, (float)cmd_AngleLeft->valueint);
    setAngle_180(&Servo_RightLeg,(float)cmd_AngleRight->valueint);

    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(3));
    cJSON_AddItemToObject(response_root, "currentAngle_Left", cJSON_CreateNumber(Servo_LeftLeg.prTarget_Angle));
    cJSON_AddItemToObject(response_root, "currentAngle_Right", cJSON_CreateNumber(Servo_RightLeg.prTarget_Angle));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_getMotorSpeed(cJSON* root){
    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(4));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_getMotorAngle(cJSON* root){
    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(5));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_getMotorCurrent(cJSON* root){
    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(6));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_getSystemMode(cJSON* root){
    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(7));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_setHeartBeat(cJSON* root){
    cJSON *cmd_beatTime_ms = cJSON_GetObjectItem(root, "beatTime_ms");
    if(cmd_beatTime_ms == NULL){
        JSON_response("Error before: [%s]\n", cJSON_GetErrorPtr());
        return;
    }
    sysLog.beatTime_ms = cmd_beatTime_ms->valueint;

    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(8));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    res_sendHeartBeat();

    cJSON_Delete(response_root);
    free(responseText);
}

void cmd_setFOCMode(cJSON *root){
    cJSON *cmd_FOCMode = cJSON_GetObjectItem(root, "FOC_Mode");
    if (!cJSON_IsNumber(cmd_FOCMode) ||
        (cmd_FOCMode->valueint != SPEED_CONTROL && cmd_FOCMode->valueint != POSITION_CONTROL)) {
        JSON_response("Error: unsupported FOC mode\r\n");
        return;
    }

    focControlMode = (FOC_CONTROL_MODE)cmd_FOCMode->valueint;

    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddItemToObject(response_root, "res", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(response_root, "cmd", cJSON_CreateNumber(9));
    cJSON_AddItemToObject(response_root, "NewFOC_Mode", cJSON_CreateNumber(focControlMode));

    char* responseText = cJSON_Print(response_root);

    JSON_response("%s", responseText);

    cJSON_Delete(response_root);
    free(responseText);
}
