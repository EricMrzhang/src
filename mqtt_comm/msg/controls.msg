string msgType
string agvId
int64 timestamp

#task
int32 taskId
int32 taskType 
string targetName 
float64 targetX
float64 targetY
float64 targetHA
float64 cargoLength
float64 cargoWidth
float64 cargoWeight
string cargoId
float64 wheelBase
int32 subtaskCount
int32 subtaskIndex
int32 subtaskAction
int32 pathLength
path_point[] path

# sync
int32 value

# video
int32 channelId
int32 cmd       
int32 timeSec
string addr

# ctrl
int32 cmdId
string ctrlType
float64 stopPosX
float64 stopPosY
float64 stopPosHA 
int32 lightStatus
int32 lightType
int32 lightLeftTime
int32 intersectionId





