# Node Communication Test

## Description

The following aspects are tested.

|Check Point No.|Description|
|---|---|
|1|The fixed-cycle transmission topics should be able to be sent at the expected intervals. In the sequence diagram of the module design document, the topics sent from own module/own node are applicable.|
|2|Received topics must be able to be received. In the sequence diagram of the module design document, the topics that own module/own node receives are applicable.|
|3|Must be able to respond to requests for service communication. In the sequence diagram of the module design document, the service provided by own module/own node corresponds.|
|4|Do not respond when not ready to accept a request for service communication. In the sequence diagram of the module design document, the service provided by own module/own node corresponds.|

* About checkpoint 4 in this module  
   The services handled by this module are ready immediately after node startup. Therefore, it is impossible to check this checkpoint.

The following are excluded from this test

* Specific request/response values included in the topic or service.
  * Confirm by testing the functionality that handles values.

## Test coverage

The correspondence between the test objects and the point of view is as follows.

|Test Target No.|Test Target|Check Point No.|
|---|---|---|
|1| `/cargo_loading/infrastructure_commands`      |1|
|2| `/in_parking/state`                           |2|
|3| `/infrastructure_status`                      |2|
|4| `/parking/cargo_loading`                      |3|

## Test procedure

Perform each step of the operation in turn. After each step is completed, check to see if the expectations described in the "After Step.X" section of the "Test report" are met.

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

### Step.2

```sh
# Terminal-3
ros2 topic echo /cargo_loading/infrastructure_commands
```

```sh
# Terminal-4
ros2 service call /parking/cargo_loading in_parking_msgs/srv/ExecuteInParkingTask "{key: '100', value: '200'}"
```


### Step.3

```sh
# Terminal-5
ros2 topic pub --once /infrastructure_status v2i_interface_msgs/msg/InfrastructureStateArray "{stamp: {sec: 1, nanosec: 1}, states: [{stamp: {sec: 1, nanosec: 1}, type: 'eva_beacon_system', id: '200', approval: true}]}"
```

## Test report

Each time the steps described in "Test procedure" are performed, check that the software behavior matches the expectations of this item.

Steps for which there are no items to check are omitted from the heading, since the preparation is divided into multiple steps.

### After Step.1

**Test Target No.2**

- Expectation
  - `/in_parking/state`
    - cargo_loading_service outputs the contents of the subscribed messages to the debug log.
- Result
  - OK

```sh
# Terminal-1
[component_container_mt-1] [DEBUG] [1676440283.026451770] [cargo_loading.cargo_loading_service]: Subscribed /in_parking/state:stamp:
[component_container_mt-1]   sec: 1
[component_container_mt-1]   nanosec: 1
[component_container_mt-1] aw_state: 3
[component_container_mt-1] vehicle_operation_mode: 1
[component_container_mt-1] on_arrived_goal: true
[component_container_mt-1] 
```

### After Step.2

**Test Target No.1**

- Expectation
  - `/cargo_loading/infrastructure_commands`
    - Send interval 200msec.
- Result
  - OK

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
---
stamp:
  sec: 1676440891
  nanosec: 575501214
commands:
- stamp:
    sec: 1676440891
    nanosec: 575501214
  type: eva_beacon_system
  id: '200'
  state: 1
```

### After Step.3

**Test Target No.3**

- Expectation
  - `/infrastructure_status`
    - cargo_loading_service outputs the contents of the subscribed messages to the debug log.
- Result
  - OK

```sh
# Terminal-1
[component_container_mt-1] [DEBUG] [1676441102.955167577] [cargo_loading.cargo_loading_service]: Subscribed /infrastructure_status:stamp:
[component_container_mt-1]   sec: 1
[component_container_mt-1]   nanosec: 1
[component_container_mt-1] states:
[component_container_mt-1] -
[component_container_mt-1]   stamp:
[component_container_mt-1]     sec: 1
[component_container_mt-1]     nanosec: 1
[component_container_mt-1]   type: "eva_beacon_system"
[component_container_mt-1]   id: "200"
[component_container_mt-1]   approval: true
[component_container_mt-1] 
```

**Test Target No.4**

- Expectation
  - `/parking/cargo_loading`
    - A response is returned to the service request.
- Result
  - OK

```sh
# Terminal-4
requester: making request: in_parking_msgs.srv.ExecuteInParkingTask_Request(key='100', value='200')

response:
in_parking_msgs.srv.ExecuteInParkingTask_Response(state=1)

```
