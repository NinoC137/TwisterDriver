//
// Created by Yoshi on 2024/3/10.
//

#ifndef TWISTERDRIVER_CMD_PARSE_H
#define TWISTERDRIVER_CMD_PARSE_H

#include "main.h"

void cmd_startParse(char* JsonString);

void res_sendHeartBeat();

void cmd_setAngularDeviation(cJSON* root);

void cmd_setServoAngle(cJSON* root);

void cmd_getMotorSpeed(cJSON* root);

void cmd_getMotorAngle(cJSON* root);

void cmd_getMotorCurrent(cJSON* root);

void cmd_getSystemMode(cJSON* root);

void cmd_setHeartBeat(cJSON* root);

#endif //TWISTERDRIVER_CMD_PARSE_H
