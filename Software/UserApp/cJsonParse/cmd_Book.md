# TwisterDriver JSON命令包示例

## 心跳包

>{   	
>   "name":"TwisterRobot v1.0",     	
>   "lastUpgrade":"2024.3.10",   	
>   "sysRunTime": 7   	
>}

## Json指令
### 	cmd_setAngularDeviation

>{  	
>   "cmd":1,  	
>   "Deviation": 5  	
>}

**返回值:**

>{  	
>   "res":0,     	
>   "cmd":1     	
>}

### 	cmd_setServoAngle

>{  	
>   "cmd":2,  	
>   "ServoAngle_Left": 5,    	
>   "ServoAngle_Right": 5  	
>}

**返回值:**
>{  	
>"res":0,     	
>"cmd":2,
>
>"currentAngle_Left": 5,    	
>"currentAngle_Right": 5  	
>
>
>}
### 	cmd_getMotorSpeed

>{  	
>   "cmd":3   	
>}

**返回值:**

>{  	
>   "res":0,     	
>   "cmd":3,    	
>   "speedLeft":5,  	
>   "speedRight":5  	
>}
### 	cmd_getMotorAngle

>{  	
>   "cmd":4   	
>}

**返回值:**
>{  	
>   "res":0,     	
>   "cmd":4,     	
>   "angleLeft":10,  	
>   "angleRight":50  	
>}
### 	cmd_getMotorCurrent

>{  	
>   "cmd":5   	
>}

**返回值:**
>{  	
>   "res":0,     	
>   "currentLeft":300,  	
>   "currentRight":300  	
>}
### 	cmd_getSystemMode

>{  	
>   "cmd":6     	
>}

**返回值:**

>{  	
>   "res":0,     	
>   "cmd":6,     	
>   "Mode": 1   	
>}

### 	cmd_setHeartBeat

>{  	
>   "cmd":7,     	
>   "beatTime_ms":300   	
>}

**返回值:**
>{  	
>   "res":0,     	
>   "cmd":7,     	
>   "name":"TwisterRobot v1.0",     	
>   "lastUpgrade":"2024.3.10",   	
>   "sysRunTime": 7   	
>}