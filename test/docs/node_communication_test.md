# Node Communication Test

## Description

The following aspects are tested.

|Check Point No.|Description|
|---|---|
|1|The fixed-cycle transmission topics should be able to be sent at the expected intervals. In the sequence diagram of the module design document, the topics sent from own module/own node are applicable.|
|2|Received topics must be able to be received. In the sequence diagram of the module design document, the topics that own module/own node receives are applicable.|
|3|Must be able to respond to requests for service communication. In the sequence diagram of the module design document, the service provided by own module/own node corresponds.|
|4|If it is not ready to accept a request for service communication, it shall not accept the service request. (The service requester must know that the service cannot be requested.) In the sequence diagram of the module design document, the service provided by own module/own node corresponds.|

## Test coverage

The correspondence between the test objects and the point of view is as follows.

|Test Target No.|Test Target|Check Point No.|
|---|---|---|
|1| `/cargo_loading/infrastructure_commands`      |1|
|2| `/in_parking/state`                           |2|
|3| `/v2i/infrastructure_states`                      |2|
|4| `/in_parking/task`                      |3|

* About checkpoint 4 in this module  
   The services handled by this module are ready immediately after node startup. Therefore, it is impossible to check this checkpoint.

The following are excluded from this test

* Specific request/response values included in the topic or service.
  * Confirm by testing the functionality that handles values.

## Preliminary Preparation

In the test procedure, rqt_publisher is used for topic distribution in /in_parking/state.
In this section, we make preliminary preparations for using rqt_publisher.

1. Start the GUI with the following command.

   ```sh
   # Terminal
   ros2 run rqt_publisher rqt_publisher
   ```

1. Set the following parameters in the launched GUI.  

   |Parameter|Value|
   |---|---|
   |Topic|/in_parking/state|
   |Type|in_parking_msgs/msg/InParkingStatus|
   |Freq.|10|

   ![rqt_publisher Image](image00.png)

1. After setting these parameters, press the + button.

1. Set the following parameters in the added /in_parking/state.

   |Parameter|Value|
   |---|---|
   |stamp.sec|time()|
   |stamp.nanosec|time() * 1e9 % 1e9|

   ![rqt_publisher Image](image01.png)

1. Check the box next to /in_parking/state.

## Test procedure

Perform each step of the operation in turn. After each step is completed, check to see if the expectations described in the "After Step.X" section of the "Test report" are met.

### Step.1

Set the following parameters in /in_parking/state in rqt_publisher.
|Parameter|Value|
|---|---|
|aw_state|3|
|vehicle_operation_mode|1|

```sh
# Terminal-1
# The log output level is changed to DEBUG in some places.
ros2 run cargo_loading_service cargo_loading_service --ros-args --log-level debug
```

### Step.2

```sh
# Terminal-2
ros2 topic echo /cargo_loading/infrastructure_commands
```

```sh
# Terminal-3
ros2 service call /in_parking/task in_parking_msgs/srv/ExecuteInParkingTask "{id: '200'}"
```

### Step.3

```sh
# Terminal-4
ros2 topic pub --once /v2i/infrastructure_states v2i_interface_msgs/msg/InfrastructureStateArray "{stamp: {sec: 1, nanosec: 1}, states: [{stamp: {sec: 1, nanosec: 1}, id: '200', state: '1'}]}"
```

### Test procedure and criteria details

#### Test procedure and criteria details of target No.2

**After Step.1**

- Expectation
  - `/in_parking/state`
    - cargo_loading_service outputs the contents of the subscribed messages to the debug log.

```sh
# Terminal-1
[DEBUG] [1681281290.237293500] [cargo_loading_service]: inParkingStatus: stamp:
  sec: 1681281290
  nanosec: 237037056
aw_state: 3
vehicle_operation_mode: 1
```


#### Test procedure and criteria details of target No.1

**After Step.2**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - Send interval 200msec.

```sh
# Terminal-2
stamp:
  sec: 1681281333
  nanosec: 353043285
commands:
- stamp:
    sec: 1681281333
    nanosec: 353043285
  id: 200
  state: 1
---
stamp:
  sec: 1681281333
  nanosec: 553111519
commands:
- stamp:
    sec: 1681281333
    nanosec: 553111519
  id: 200
  state: 1
---
```

#### Test procedure and criteria details of target No.3

**After Step.3**

- Expectation
  - `/v2i/infrastructure_states`
    - cargo_loading_service outputs the contents of the subscribed messages to the debug log.

```sh
# Terminal-1
[DEBUG] [1681281569.446458085] [cargo_loading_service]: InfrastructureStatus: stamp:
  sec: 1
  nanosec: 1
states:
-
  stamp:
    sec: 1
    nanosec: 1
  id: 200
  state: 1
```

#### Test procedure and criteria details of target No.4

**After Step.3**

- Expectation
  - `/in_parking/task`
    - A response is returned to the service request.

```sh
# Terminal-3
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(id=200)

response:
in_parking_msgs.srv.ExecuteInParkingTask_Response(state=1)

```
