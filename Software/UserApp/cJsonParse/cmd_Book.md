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

### cmd_setMotorAngle

> {  	
> "cmd":2,  	
> "MotorAngle_Left": 5,    	
> "MotorAngle_Right": 5  	
> }

**返回值:**

> {  	
> "res":0,     	
> "cmd":2,
>
> "NewMotorAngle_Left": 5,    	
> "NewMotorAngle_Right": 5  	
>
> }

### 	cmd_setServoAngle

>{  	
>   "cmd":3,  	
>   "ServoAngle_Left": 5,    	
>   "ServoAngle_Right": 5  	
>}

**返回值:**

>{  	
>"res":0,     	
>"cmd":3,
>
>"currentAngle_Left": 5,    	
>"currentAngle_Right": 5  	
>
>}
>	

### cmd_getMotorSpeed

>{  	
>   "cmd":4   	
>}

**返回值:**

>{  	
>"res":0,     	
>"cmd":4,    	
>"speedLeft":5,  	
>"speedRight":5  	
>}
>
### 	cmd_getMotorAngle

>{  	
>   "cmd":5   	
>}

**返回值:**

>{  	
>"res":0,     	
>"cmd":5,     	
>"angleLeft":10,  	
>"angleRight":50  	
>}
>
### 	cmd_getMotorCurrent

>{  	
>   "cmd":6   	
>}

**返回值:**
>{  	
>   "res":0,     	
>   "currentLeft":300,  	
>   "currentRight":300  	
>}
### 	cmd_getSystemMode

>{  	
>   "cmd":7     	
>}

**返回值:**

>{  	
>   "res":0,     	
>   "cmd":7,     	
>   "Mode": 1   	
>}

### 	cmd_setHeartBeat

>{  	
>   "cmd":8,     	
>   "beatTime_ms":300   	
>}

**返回值:**
>{  	
>   "res":0,     	
>   "cmd":8,     	
>   "name":"TwisterRobot v1.0",     	
>   "lastUpgrade":"2024.3.10",   	
>   "sysRunTime": 7   	
>}

### cmd_setFOCMode

> {  	
> "cmd":9,     	
> "FOC_Mode":1   	
> }

**返回值:**

> {  	
> "res":0,     	
> "cmd":8,     	
>  "NewFOC_Mode":1
> }