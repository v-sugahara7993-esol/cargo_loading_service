# Cargo Loading Service Test

## Description

The following aspects are tested.

|Check Point No.|Description|
|---|---|
|1|The service `/parking/cargo_loading` returns success or failure as the response value.|
|2|Service behavior changes depending on the value of `aw_state` received at `/in_parking/state`.|
|3|Service behavior changes depending on the value of `approval` received in `/infrastructure_status`.|
|4|Service behavior changes depending on the value of `id` received in `/infrastructure_status`.|


## Test coverage

The correspondence between the test objects and the point of view is as follows.

|Test Target No.|Test Target|Check Point No.|
|---|---|---|
|1 | Return SUCCESS(=1)                               |1|
|2 | Return FAIL(=2)                                  |1|
|3 | aw_state is NONE(=0)                             |2|
|4 | aw_state is AW_EMERGENCY(=1)                     |2|
|5 | aw_state is AW_OUT_OF_PARKING(=2)                |2|
|6 | aw_state is AW_ARRIVED_PARKING(=3)               |2|
|7 | aw_state is AW_INVALID_DEPARTURE_INSTRUCTION(=4) |2|
|8 | aw_state is AW_WAIT_DEPARTURE_INSTRUCTION(=5)    |2|
|9 | aw_state is AW_UNAVAILABLE(=6)                   |2|
|10| approval is true                                 |3|
|11| approval is false                                |3|
|12| Match with ID requested by service               |4|
|13| Mismatch with ID requested by service            |4|

### Common test procedure

### Step.1

```sh
# Terminal-1
# The log output level is changed to DEBUG in some places.
ros2 launch cargo_loading_service cargo_loading_service.launch.py log-level:=debug
```

```sh
# Terminal-2
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 3, vehicle_operation_mode: 1}"
```

```sh
# Terminal-3
ros2 topic echo /cargo_loading/infrastructure_commands
```

```sh
# Terminal-4
ros2 service call /parking/cargo_loading in_parking_msgs/srv/ExecuteInParkingTask "{key: '100', value: '200'}"
```

```sh
# Terminal-5
ros2 topic pub --once /infrastructure_status v2i_interface_msgs/msg/InfrastructureStateArray "{stamp: {sec: 1, nanosec: 1}, states: [{stamp: {sec: 1, nanosec: 1}, type: 'eva_beacon_system', id: '200', approval: true}]}"
```

### Step.2

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 0, vehicle_operation_mode: 1}"
```

```sh
# Terminal-4
ros2 service call /parking/cargo_loading in_parking_msgs/srv/ExecuteInParkingTask "{key: '100', value: '200'}"
```

### Step.3

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 4, vehicle_operation_mode: 1}"
```

```sh
# Terminal-4
ros2 service call /parking/cargo_loading in_parking_msgs/srv/ExecuteInParkingTask "{key: '100', value: '200'}"
```

### Step.4

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 5, vehicle_operation_mode: 1}"
```

### Step.5

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 2, vehicle_operation_mode: 1}"
```

### Step.6

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 3, vehicle_operation_mode: 1}"
```

```sh
# Terminal-4
ros2 service call /parking/cargo_loading in_parking_msgs/srv/ExecuteInParkingTask "{key: '100', value: '200'}"
```

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 6, vehicle_operation_mode: 1}"
```

### Step.7

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 3, vehicle_operation_mode: 1}"
```

```sh
# Terminal-4
ros2 service call /parking/cargo_loading in_parking_msgs/srv/ExecuteInParkingTask "{key: '100', value: '200'}"
```

```sh
# Terminal-5
ros2 topic pub --once /infrastructure_status v2i_interface_msgs/msg/InfrastructureStateArray "{stamp: {sec: 1, nanosec: 1}, states: [{stamp: {sec: 1, nanosec: 1}, type: 'eva_beacon_system', id: '200', approval: false}]}"
```

```sh
# Terminal-5
ros2 topic pub --once /infrastructure_status v2i_interface_msgs/msg/InfrastructureStateArray "{stamp: {sec: 1, nanosec: 1}, states: [{stamp: {sec: 1, nanosec: 1}, type: 'eva_beacon_system', id: '201', approval: true}]}"
```

### Step.8

```sh
# Terminal-2
# Stop commands running in this terminal and execute the following commands.
ros2 topic pub /in_parking/state in_parking_msgs/msg/InParkingStatus "{stamp: {sec: 1, nanosec: 1}, aw_state: 1, vehicle_operation_mode: 1}"
```

```sh
# Terminal-5
ros2 topic pub --once /infrastructure_status v2i_interface_msgs/msg/InfrastructureStateArray "{stamp: {sec: 1, nanosec: 1}, states: [{stamp: {sec: 1, nanosec: 1}, type: 'eva_beacon_system', id: '200', approval: true}]}"
```

### Test procedure and criteria details

#### Test procedure and criteria details of target No.1

**After Step 1**

- Expectation
  - `/parking/cargo_loading`
    - Returns a successful response to a service request.
      - The state of the response message must be 1.

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

response:
in_parking_msgs.srv.ExecuteInParkingTask_Response(state=1)

```

#### Test procedure and criteria details of target No.2

**After Step 2**

- Expectation
  - `/parking/cargo_loading`
    - Returns a failure response to a service request.
      - The state of the response message must be 2.

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

response:
in_parking_msgs.srv.ExecuteInParkingTask_Response(state=2)

```

#### Test procedure and criteria details of target No.3

**After Step 2**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - Not publishing a message on the topic.

```sh
# Terminal-3
# Confirm that no new messages have been received.

```

#### Test procedure and criteria details of target No.4

**After Step 8**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 2 must be continuously published.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 2
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 2
```

#### Test procedure and criteria details of target No.5

**After Step 5**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 254 must be continuously published.
    - Stopping publishing after 2 seconds.
  - `/parking/cargo_loading`
    - Returns a successful response to a service request.
      - The state of the response message must be 1.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 254
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 254
```

#### Test procedure and criteria details of target No.6

**After Step 1**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 1 must be continuously published.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 1
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 1
```

#### Test procedure and criteria details of target No.7

**After Step 3**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 1 must be continuously published.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 1
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 1
```

#### Test procedure and criteria details of target No.8

**After Step 4**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 1 must be continuously published.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 1
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 1
```

#### Test procedure and criteria details of target No.9

**After Step 6**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 254 must be continuously published.
    - Stopping publishing after 2 seconds.
  - `/parking/cargo_loading`
    - Returns a successful response to a service request.
      - The state of the response message must be 1.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 254
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 254
```

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

response:
in_parking_msgs.srv.ExecuteInParkingTask_Response(state=1)

```

#### Test procedure and criteria details of target No.10

**After Step 1**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 254 must be continuously published.
    - Stopping publishing after 2 seconds.
  - `/parking/cargo_loading`
    - Returns a successful response to a service request.
      - The state of the response message must be 1.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 0
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 254
```

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

response:
in_parking_msgs.srv.ExecuteInParkingTask_Response(state=1)

```

#### Test procedure and criteria details of target No.11

**After Step 7**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - The state of the message being published should not change.
  - `/parking/cargo_loading`
    - Not returning service requests.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 1
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 1
```

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

```

#### Test procedure and criteria details of target No.12

**After Step 1**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - A message with a state of 254 must be continuously published.
    - Stopping publishing after 2 seconds.
  - `/parking/cargo_loading`
    - Returns a successful response to a service request.
      - The state of the response message must be 1.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 254
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 254
```

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

response:
in_parking_msgs.srv.ExecuteInParkingTask_Response(state=1)

```

#### Test procedure and criteria details of target No.13

**After Step 8**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - The state of the message being published should not change.
  - `/parking/cargo_loading`
    - Not returning service requests.

```sh
# Terminal-3
stamp:
  sec: 1676440891
  nanosec: 175108980
commands:
- stamp:
    sec: 1676440891
    nanosec: 175108980
  type: eva_beacon_system
  id: '200'
  state: 1
---
stamp:
  sec: 1676440891
  nanosec: 375340288
commands:
- stamp:
    sec: 1676440891
    nanosec: 375340288
  type: eva_beacon_system
  id: '200'
  state: 1
```

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

```

