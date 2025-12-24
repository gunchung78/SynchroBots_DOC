import asyncio
import json         # JSON 문자열을 딕셔너리로 변환하기 위해 추가
import httpx        # HTTP 요청을 위한 라이브러리
from asyncua import Server, ua
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
import threading

# --- 전역 설정 ---
# Web PC의 HTTP 엔드포인트 URL 설정
# (실제 배포 시 IP와 PORT를 변경해야 합니다.)
IP = "172.10.1.5"
PORT = "8080"
WEB_SERVER_URL = "http://{0}:{1}/api/v1/amr/getAmrStatusCheck".format(IP, PORT)
WEB_IMAGE_REPORT_URL = "http://{0}:{1}/api/v1/robotarm/send_arm_img".format(IP, PORT) # ARM_001

# --- Modbus TCP 설정 ---
# PLC_002 결과를 저장할 Modbus Holding Register. 주소는 80 (인덱스 0)
MODBUS_REGISTERS = {
    80 : 0  # 0: NORMAL/CONTINUE, 1: ANOMALY/STOP
}
store = ModbusSlaveContext(
    hr=ModbusSequentialDataBlock(0, [0] * 100)
)
modbus_context = ModbusServerContext(slaves=store, single=True)

# ------------------------------------------------------------------------------------- #

# OPC UA 메소드 구현을 위한 비동기 클래스 정의
# OPC UA 메소드는 서버의 내부 로직을 OPC UA를 통해 원격으로 호출하는 방식입니다.
class ServerMethods:
    def __init__(self, server_instance, idx):
        self.server = server_instance
        self.idx = idx
        self.objects_node = self.server.nodes.objects
        self.read_amr_go_move_node = None                   # AMR_001 결과 반영 노드
        self.read_amr_go_positions_node = None              # AMR_002 결과 반영 노드
        self.read_amr_mission_state_node = None             # AMR_003 결과 반영 노드

        self.read_converyor_sensor_cheak_node = None        # PLC_001 결과 반영 노드
        self.read_ok_ng_value_node = None                   # PLC_002 결과 반영 노드
        self.read_robotarm_sensor_cheak_node = None         # PLC_003 결과 반영 노드
        self.read_ready_state_node = None                   # PLC_004 결과 반영 노드

        self.read_arm_img_node = None                       # ARM_01 결과 반영 노드

    async def init_nodes(self):
        """데이터를 수신 시스템에 노출하기 위한 Read 전용 노드 정의"""
        my_objects = await self.objects_node.add_object(self.idx, "InterfaceDataNodes")
        
        # --- [AMR_001] 수신 시스템(AMR)이 읽을 노드 ---
        # WEB이 보낸 명령의 처리 상태를 AMR이 모니터링하기 위한 노드
        self.read_amr_go_move_node = await my_objects.add_variable(self.idx, "read_amr_go_move_status", "READY", datatype=ua.NodeId(ua.ObjectIds.String))

        # --- [AMR_002] 수신 시스템(AMR)이 읽을 노드 ---
        # WEB이 보낸 명령의 처리 상태를 AMR이 모니터링하기 위한 노드
        self.read_amr_go_positions_node = await my_objects.add_variable(self.idx, "read_amr_go_positions_status", "READY", datatype=ua.NodeId(ua.ObjectIds.String))
        
        # --- [AMR_003] 수신 시스템(WEB)이 읽을 노드 ---
        # WEB이 보낸 명령의 처리 상태를 AMR이 모니터링하기 위한 노드
        self.read_amr_mission_state_node = await my_objects.add_variable(self.idx, "read_amr_mission_state_status", "READY", datatype=ua.NodeId(ua.ObjectIds.String))

        # --- [PLC_001] 수신 시스템(WEB)이 읽을 노드 ---
        # PLC의 센서 체크 결과를 WEB이 모니터링하기 위한 노드
        self.read_converyor_sensor_cheak_node = await my_objects.add_variable(self.idx, "read_conveyor_sensor_check_status", "READY", datatype=ua.NodeId(ua.ObjectIds.String))

        # --- [PLC_002] 수신 시스템(PLC)이 읽을 노드 ---
        # WEB의 이상 유무 판별 결과를 PLC가 모니터링하기 위한 노드
        # 이 OPC UA 노드는 이제 Modbus 매핑의 최종 상태를 나타냅니다.
        global MODBUS_REGISTERS
        initial_status = "Modbus Register 80 Mapped: 0 (NORMAL)"
        self.read_ok_ng_value_node = await my_objects.add_variable(self.idx, "read_ok_ng_value_modbus_status", initial_status, datatype=ua.NodeId(ua.ObjectIds.String))

        # --- [PLC_003] 수신 시스템(WEB)이 읽을 노드 ---
        # PLC의 로봇 팔 센서 체크 결과를 WEB이 모니터링하기 위한 노드 (PLC_001과 로직 동일)
        self.read_robotarm_sensor_cheak_node = await my_objects.add_variable(self.idx, "read_robotarm_sensor_check_status", "READY", datatype=ua.NodeId(ua.ObjectIds.String))

        # --- [PLC_004] 수신 시스템(PLC)이 읽을 노드 ---
        # WEB이 보낸 동작 완료/계속 명령 상태를 PLC가 모니터링하기 위한 노드
        self.read_ready_state_node = await my_objects.add_variable(self.idx, "read_ready_status", "AWAITING_COMMAND", datatype=ua.NodeId(ua.ObjectIds.String))

        # --- [ARM_001] (ARM -> WEB) 이미지 전송 HTTP 위임 상태 ---
        self.read_arm_img_node = await my_objects.add_variable(self.idx, "read_arm_img_node_status", "READY", datatype=ua.NodeId(ua.ObjectIds.String))
        
        return my_objects
    
    # -----------------------------------------------------
    # AMR_001 인터페이스 로직 (Web PC -> AMR)
    # -----------------------------------------------------
    async def call_amr_go_move(self, parent_node, json_command_str):
        """
        Web PC가 호출하는 OPC UA Method. AMR의 이동 명령을 처리합니다.
        Input: json_command_str (String) - JSON 문자열 e.g., '{"move_command": "go_home"}'
        Output: (Int, String) - (결과 코드, 메시지)
        """
        
        try:
            # 1. JSON 문자열을 Python 딕셔너리로 역직렬화 (Deserialization)
            command_data = json.loads(json_command_str)
            move_command = command_data.get("move_command")
        except json.JSONDecodeError:
            await self.read_amr_go_move_node.set_value(f"Error: Invalid JSON format.")
            return 1, f"Error: Invalid JSON format received."
        except Exception:
            await self.read_amr_go_move_node.set_value(f"Error: Missing 'move_command' key in JSON.")
            return 1, f"Error: Missing 'move_command' key."


        # 2. 데이터 유효성 검사 및 변환 로직
        if move_command not in ["go_home", "pick_up_zone", "stop"]:
            await self.read_amr_go_move_node.set_value(f"Error: Invalid command ({move_command})")
            return 1, f"Error: Invalid command ({move_command})" # 결과 코드 1: 오류

        # 3. '데이터 변환' 및 처리 시뮬레이션
        await self.read_amr_go_move_node.set_value(f"Processing command: {move_command}")
        await asyncio.sleep(0.5) # 실제 AMR 명령 하달 및 처리 시간 시뮬레이션

        # 4. 수신 시스템(AMR)이 모니터링할 노드에 최종 상태 반영
        await self.read_amr_go_move_node.set_value(f"Command Executed: {move_command}")
        
        # 5. Method 호출자에게 성공 응답 반환
        return 0, f"Success: Command '{move_command}' relayed to AMR." # 결과 코드 0: 성공
    
    # -----------------------------------------------------
    # AMR_002 인터페이스 로직 (Web -> AMR)
    # -----------------------------------------------------
    async def call_amr_go_positions(self, parent_node, json_object_info_str):
        """
        Web PC가 호출하는 OPC UA Method. 로봇 팔 오브젝트 정보 (JSON 리스트)를 처리합니다.
        Input: json_object_info_str (String) - JSON 문자열 e.g., '{"object_info": ["esp32", "motordriver", "powersuplpy]}'
        Output: (Int, String) - (결과 코드, 메시지)
        """
        try:
            # 1. JSON 문자열을 Python 딕셔너리로 역직렬화
            info_data = json.loads(json_object_info_str)
            object_info = info_data.get("object_info")
        except json.JSONDecodeError:
            await self.read_amr_go_positions_node.set_value(f"Error: Invalid JSON format.")
            return 1, f"Error: Invalid JSON format received."
        except Exception:
            await self.read_amr_go_positions_node.set_value(f"Error: Missing 'object_info' key in JSON.")
            return 1, f"Error: Missing 'object_info' key."

        # 2. 데이터 유효성 검사 (리스트 여부 및 길이 확인)
        if not isinstance(object_info, list) or len(object_info) == 0:
            await self.read_amr_go_positions_node.set_value(f"Error: 'object_info' must be a non-empty list.")
            return 1, f"Error: 'object_info' must be a non-empty list."

        # 3. '데이터 변환' 및 처리 시뮬레이션
        # 리스트 내용을 요약하여 상태 메시지로 변환
        summary = f"Received {len(object_info)} items: {object_info[0]}..."
        await self.read_amr_go_positions_node.set_value(f"Processing Object Info: {summary}")
        await asyncio.sleep(0.7) # 처리 시간 시뮬레이션

        # 4. 수신 시스템(AMR)이 모니터링할 노드에 최종 상태 반영
        await self.read_amr_go_positions_node.set_value(f"Object Info Processed. Total items: {len(object_info)}")
        
        # 5. Method 호출자에게 성공 응답 반환
        return 0, f"Success: Object info list ({len(object_info)} items) relayed to AMR."

    # -----------------------------------------------------
    # AMR_003 인터페이스 로직 (AMR -> WEB)
    # -----------------------------------------------------
    async def call_amr_mission_state(self, parent_node, json_mission_state_str):
        """
        AMR이 호출하는 OPC UA Method. 임무 상태 정보를 Web PC의 HTTP 엔드포인트로 POST 전송합니다.
        Input: json_mission_state_str (String) - JSON 문자열 (예: {"equipment_id": "AMR_1", "mission_id": "AMR_1_251125", "status": "DONE"})
        Output: (Int, String) - (결과 코드, 메시지)
        """
        
        try:
            # 1. JSON 문자열을 Python 딕셔너리로 역직렬화 및 유효성 검사
            mission_data = json.loads(json_mission_state_str)
            # AMR_003 정의서에 따른 필수 컬럼 'status' 확인
            if 'status' not in mission_data:
                 raise ValueError("Missing 'status' key in JSON.")

            # 2. Web PC의 HTTP 엔드포인트로 POST 요청 전송
            # HTTP 요청은 블로킹(Blocking)을 피하기 위해 httpx 라이브러리를 사용합니다.
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    WEB_SERVER_URL,  # 전역 변수 WEB_SERVER_URL 사용
                    json=mission_data,
                    headers={"Content-Type": "application/json"}
                )

            # 3. HTTP 응답 상태 확인
            if response.status_code == 200:
                result_message = f"Success: Mission state POST to WEB PC. HTTP Status: {response.status_code}"
                result_code = 0
            else:
                result_message = f"HTTP Error: Failed to POST to WEB PC. Status: {response.status_code}, Response: {response.text}"
                result_code = 1

        except json.JSONDecodeError:
            result_code = 1
            result_message = f"Error: Invalid JSON format received."
        except ValueError as ve:
            result_code = 1
            result_message = f"Error: {ve}"
        except httpx.RequestError as re:
            result_code = 1
            result_message = f"Network Error: Failed to connect to WEB PC at {WEB_SERVER_URL}. ({re})"
        except Exception as e:
            result_code = 1
            result_message = f"Unhandled Error: {e}"

        # 4. 수신 시스템(WEB)이 모니터링할 노드에 최종 상태 반영
        await self.read_amr_mission_state_node.set_value(f"Last Status: {json_mission_state_str} | Result: {result_message}")

        # 5. Method 호출자(AMR)에게 응답 반환
        return result_code, result_message
    
    # -----------------------------------------------------
    # PLC_001 인터페이스 로직 (PLC -> WEB)
    # -----------------------------------------------------
    async def call_set_conveyorSensor_check(self, parent_node, conveyorSensor_check):
        """
        PLC가 호출하는 OPC UA Method. 컨베이어 센서 체크 신호를 처리합니다.
        Input: conveyorSensor_check (Boolean)
        Output: (Boolean, String) - (성공 여부, 메시지)
        """
        
        # 1. 수신된 신호 로깅 (제거됨)
        
        # 2. 데이터 변환 로직 (Boolean -> 상태 문자열)
        if conveyorSensor_check is True:
            status_message = "Sensor Check OK. Conveyor Ready."
        else:
            status_message = "Sensor Check NG. Camera Triggered for Inspection."
            
        # 3. '데이터 변환' 및 처리 시뮬레이션
        # PLC 신호를 받아 WEB PC가 읽을 수 있는 상태 메시지로 변환하여 반영
        await self.read_converyor_sensor_cheak_node.set_value(status_message)
        
        # 4. Method 호출자에게 성공 응답 반환
        return True, "Success: Sensor check signal processed."

    # -----------------------------------------------------
    # PLC_002 인터페이스 로직 (WEB -> PLC)
    # -----------------------------------------------------
    async def call_ok_ng_value(self, parent_node, json_anomaly_str):
        """
        WEB PC가 호출하는 OPC UA Method. 이상 유무 판별 결과를 Modbus Register에 기록합니다.
        Input: json_anomaly_str (String) - JSON 문자열 e.g., '{"Anomaly": true}'
        Output: (Int, String) - (결과 코드, 메시지)
        """
        global MODBUS_REGISTERS, modbus_context

        # Modbus HR 인덱스 (예: 40081 → 내부 인덱스 80)
        modbus_register_address = 80

        # 1. JSON 파싱
        try:
            anomaly_data = json.loads(json_anomaly_str)
        except json.JSONDecodeError:
            await self.read_ok_ng_value_node.set_value("Error: Invalid JSON format.")
            return 1, "Error: Invalid JSON format received."

        # 1-1. 키 존재 여부 검사
        if "Anomaly" not in anomaly_data:
            await self.read_ok_ng_value_node.set_value("Error: Missing 'Anomaly' key in JSON.")
            return 1, "Error: Missing 'Anomaly' key."

        anomaly_status = anomaly_data["Anomaly"]

        # 2. 데이터 유효성 검사 (Boolean 타입 확인)
        if not isinstance(anomaly_status, bool):
            await self.read_ok_ng_value_node.set_value("Error: 'Anomaly' must be a boolean.")
            return 1, "Error: 'Anomaly' must be a boolean."

        # 3. Boolean -> Modbus 정수 값 변환
        if anomaly_status:
            modbus_value = 1  # ANOMALY -> STOP
            status_message = "ANOMALY DETECTED. Modbus Value: 1"
        else:
            modbus_value = 0  # NORMAL -> CONTINUE
            status_message = "NORMAL. Modbus Value: 0"

        # 4-1. 디버그용 딕셔너리에도 기록
        MODBUS_REGISTERS[modbus_register_address] = modbus_value

        # 4-2. 실제 Modbus Holding Register에 값 쓰기
        #  - function code 3 (Holding Register)
        #  - single=True로 만들었으니 slave_id는 아무 값이나 가능하지만 0x00로 통일
        slave_id = 0x03
        modbus_context[slave_id].setValues(3, modbus_register_address, [modbus_value])

        # 5. OPC UA 모니터링 노드 업데이트
        await self.read_ok_ng_value_node.set_value(
            f"[{status_message}] -> HR[{modbus_register_address}] = {modbus_value}"
        )
        await asyncio.sleep(0.3)

        # 6. Method 호출자에게 성공 응답 반환
        return 0, (
            f"Success: Anomaly status mapped to Modbus Register {modbus_register_address}. "
            f"Value: {modbus_value}"
        )
    
    # -----------------------------------------------------
    # PLC_003 인터페이스 로직 (PLC -> WEB) - PLC_001과 로직 동일
    # -----------------------------------------------------
    async def call_set_robotArmSensor_check(self, parent_node, robotArmSensor_check):
        """
        PLC가 호출하는 OPC UA Method. 로봇 팔 센서 체크 신호를 처리합니다.
        Input: robotArmSensor_check (Boolean)
        Output: (Boolean, String) - (성공 여부, 메시지)
        """
        
        # 1. 수신된 신호 로깅 (제거됨)
        
        # 2. 데이터 변환 로직 (Boolean -> 상태 문자열)
        if robotArmSensor_check is True:
            status_message = "Robot Arm Sensor Check OK. Ready for operation."
        else:
            status_message = "Robot Arm Sensor Check NG. Maintenance required."
            
        # 3. '데이터 변환' 및 처리 시뮬레이션
        # PLC 신호를 받아 WEB PC가 읽을 수 있는 상태 메시지로 변환하여 반영
        await self.read_robotarm_sensor_cheak_node.set_value(status_message)
        
        # 4. Method 호출자에게 성공 응답 반환
        return True, "Success: Robot Arm Sensor check signal processed."

    # -----------------------------------------------------
    # PLC_004 인터페이스 로직 (WEB -> PLC: 동작 완료 신호 전달)
    # -----------------------------------------------------
    async def call_ready_state(self, parent_node, json_state_str):
        """
        WEB PC가 호출하는 OPC UA Method. 로봇 팔 동작 완료 후 PLC에게 다음 동작 명령을 전달합니다.
        Input: json_state_str (String) - JSON 문자열 e.g., '{"state": "CYCLE_COMPLETE"}'
        Output: (Int, String) - (결과 코드, 메시지)
        """
        
        try:
            # 1. JSON 문자열을 Python 딕셔너리로 역직렬화
            state_data = json.loads(json_state_str)
            state_command = state_data.get("state")
        except json.JSONDecodeError:
            await self.read_ready_state_node.set_value(f"Error: Invalid JSON format.")
            return 1, f"Error: Invalid JSON format received."
        except Exception:
            await self.read_ready_state_node.set_value(f"Error: Missing 'state' key in JSON.")
            return 1, f"Error: Missing 'state' key."

        # 2. 데이터 유효성 검사 및 변환 로직
        if state_command not in ["CYCLE_COMPLETE", "CONTINUE", "PAUSE"]:
            await self.read_ready_state_node.set_value(f"Error: Invalid state command: {state_command}")
            return 1, f"Error: Invalid state command: {state_command}"

        # 3. '데이터 변환' 및 처리 시뮬레이션
        # Web PC 명령을 PLC가 인지할 수 있는 상세 상태로 변환
        if state_command == "CYCLE_COMPLETE":
            status_message = "ARM_CYCLE_COMPLETE. PLC: START CONVEYOR"
        else:
            status_message = f"Received Command: {state_command}"

        await self.read_ready_state_node.set_value(f"Processing Command: {status_message}")
        await asyncio.sleep(0.3) # 처리 시간 시뮬레이션

        # 4. 수신 시스템(PLC)이 모니터링할 노드에 최종 상태 반영
        await self.read_ready_state_node.set_value(status_message)
        
        # 5. Method 호출자에게 성공 응답 반환
        return 0, f"Success: State '{state_command}' relayed to PLC."
    
    # -----------------------------------------------------
    # ARM_001 인터페이스 로직 (ARM -> WEB) - 이미지 HTTP POST 위임
    # -----------------------------------------------------
    async def call_send_arm_img(self, parent_node, json_img_data_str):
        """
        ARM이 호출하는 OPC UA Method. Base64 인코딩된 이미지 JSON을 Web PC로 POST 위임합니다.
        """
        await self.read_arm_img_node.set_value(f"Processing Image Report via HTTP...")
        
        try:
            async with httpx.AsyncClient(timeout=20.0) as client: # 이미지 전송은 시간이 더 걸릴 수 있음
                response = await client.post(
                    WEB_IMAGE_REPORT_URL,
                    content=json_img_data_str,
                    headers={"Content-Type": "application/json"}
                )

                if 200 <= response.status_code < 300:
                    status_msg = f"Image Report Success (Code: {response.status_code})."
                    result_code = 0
                else:
                    status_msg = f"Image Report Failed (Code: {response.status_code})."
                    result_code = 1
                
                await self.read_arm_img_node.set_value(status_msg)
                return result_code, status_msg

        except Exception as e:
            error_msg = f"HTTP Connection Error during Image Report: {e.__class__.__name__}."
            await self.read_arm_img_node.set_value(error_msg)
            return 1, error_msg

# ----------------------------------v-------------------
# Helper 함수: Method Arguments 정의
# -----------------------------------------------------
def define_amr_001_arguments():
    """AMR_001 (amr_go_move) 메소드의 입/출력 인수를 정의합니다."""
    # Input Argument: JSON 문자열 (String 타입으로 전송)
    input_arg = ua.Argument()
    input_arg.Name = "json_command_str"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.String)
    input_arg.ValueRank = -1
    input_arg.Description = ua.LocalizedText("AMR 이동 명령을 담은 JSON 문자열 (예: {'move_command': 'go_home'})")
    
    # Output Argument 1: ResultCode (Int32)
    output_arg_1 = ua.Argument()
    output_arg_1.Name = "ResultCode"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Int32)
    output_arg_1.ValueRank = -1
    output_arg_1.Description = ua.LocalizedText("처리 결과 코드 (0: 성공, 1: 오류)")
    
    # Output Argument 2: ResultMessage (String)
    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.ValueRank = -1
    output_arg_2.Description = ua.LocalizedText("처리 상세 메시지")
    
    return [input_arg], [output_arg_1, output_arg_2]

def define_amr_002_arguments():
    """AMR_002 (amr_go_positions) 메소드의 입/출력 인수를 정의합니다."""
    # Input Argument: json_object_info_str (String)
    input_arg = ua.Argument()
    input_arg.Name = "json_object_info_str"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.String)
    input_arg.ValueRank = -1
    input_arg.Description = ua.LocalizedText("오브젝트 정보 리스트를 포함하는 JSON 문자열 (e.g., {'object_info': ['item1', 'item2']})")
    
    # Output Argument 1: ResultCode (Int32)
    output_arg_1 = ua.Argument()
    output_arg_1.Name = "ResultCode"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Int32)
    output_arg_1.ValueRank = -1
    output_arg_1.Description = ua.LocalizedText("처리 결과 코드 (0: 성공, 1: 오류)")
    
    # Output Argument 2: ResultMessage (String)
    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.ValueRank = -1
    output_arg_2.Description = ua.LocalizedText("처리 상세 메시지")
    
    return [input_arg], [output_arg_1, output_arg_2]

def define_amr_003_arguments():
    """AMR_003 (amr_mission_state) 메소드의 입/출력 인수를 정의합니다."""
    # Input Argument: json_mission_state_str (String)
    input_arg = ua.Argument()
    input_arg.Name = "json_mission_state_str"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.String)
    input_arg.ValueRank = -1
    input_arg.Description = ua.LocalizedText("AMR 임무 상태 정보 JSON 문자열 (e.g., {'equipment_id': 'AMR_1', 'status': 'DONE'})")
    
    # Output Argument 1: ResultCode (Int32)
    output_arg_1 = ua.Argument()
    output_arg_1.Name = "ResultCode"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Int32)
    output_arg_1.ValueRank = -1
    output_arg_1.Description = ua.LocalizedText("처리 결과 코드 (0: 성공, 1: 오류)")
    
    # Output Argument 2: ResultMessage (String)
    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.ValueRank = -1
    output_arg_2.Description = ua.LocalizedText("처리 상세 메시지")
    
    return [input_arg], [output_arg_1, output_arg_2]

def define_plc_001_arguments():
    """PLC_001 (set_conveyorSensor_check) 메소드의 입/출력 인수를 정의합니다."""
    # Input Argument: conveyorSensor_check (Boolean)
    input_arg = ua.Argument()
    input_arg.Name = "conveyorSensor_check"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.Boolean)
    input_arg.ValueRank = -1
    input_arg.Description = ua.LocalizedText("PLC 센서 감지 신호 (True/False)")

    # Output Argument 1: Success (Boolean)
    output_arg_1 = ua.Argument()
    output_arg_1.Name = "Success"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Boolean)
    output_arg_1.ValueRank = -1
    output_arg_1.Description = ua.LocalizedText("Method 호출 성공 여부")
    
    # Output Argument 2: ResultMessage (String)
    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.ValueRank = -1
    output_arg_2.Description = ua.LocalizedText("처리 상세 메시지")

    return [input_arg], [output_arg_1, output_arg_2]

def define_plc_002_arguments():
    """PLC_002 (OK_NG_Value) 메소드의 입/출력 인수를 정의합니다."""
    # Input Argument: json_anomaly_str (String)
    input_arg = ua.Argument()
    input_arg.Name = "json_anomaly_str"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.String)
    input_arg.ValueRank = -1
    input_arg.Description = ua.LocalizedText("이상 유무 판별 결과를 담은 JSON 문자열 (예: {'Anomaly': true})")

    # Output Argument 1: ResultCode (Int32)
    output_arg_1 = ua.Argument()
    output_arg_1.Name = "ResultCode"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Int32)
    output_arg_1.ValueRank = -1
    output_arg_1.Description = ua.LocalizedText("처리 결과 코드 (0: 성공, 1: 오류)")
    
    # Output Argument 2: ResultMessage (String)
    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.ValueRank = -1
    output_arg_2.Description = ua.LocalizedText("처리 상세 메시지")

    return [input_arg], [output_arg_1, output_arg_2]

def define_plc_003_arguments():
    """PLC_003 (OK_NG_Value) 메소드의 입/출력 인수를 정의합니다."""
    # Input Argument: robotArmSensor_check (Boolean)
    input_arg = ua.Argument()
    input_arg.Name = "robotArmSensor_check"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.Boolean)
    input_arg.ValueRank = -1
    input_arg.Description = ua.LocalizedText("PLC 로봇 팔 센서 감지 신호 (True/False)")

    # Output Argument 1: Success (Boolean)
    output_arg_1 = ua.Argument()
    output_arg_1.Name = "Success"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Boolean)
    output_arg_1.ValueRank = -1
    output_arg_1.Description = ua.LocalizedText("Method 호출 성공 여부")
    
    # Output Argument 2: ResultMessage (String)
    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.ValueRank = -1
    output_arg_2.Description = ua.LocalizedText("처리 상세 메시지")

    return [input_arg], [output_arg_1, output_arg_2]

def define_plc_004_arguments():
    """PLC_004 (Ready_State) 메소드의 입/출력 인수를 정의합니다."""
    # Input Argument: json_state_str (String)
    input_arg = ua.Argument()
    input_arg.Name = "json_state_str"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.String)
    input_arg.ValueRank = -1
    input_arg.Description = ua.LocalizedText("로봇 팔 동작 완료 명령을 담은 JSON 문자열 (e.g., {'state': 'CYCLE_COMPLETE'})")

    # Output Argument 1: ResultCode (Int32)
    output_arg_1 = ua.Argument()
    output_arg_1.Name = "ResultCode"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Int32)
    output_arg_1.ValueRank = -1
    output_arg_1.Description = ua.LocalizedText("처리 결과 코드 (0: 성공, 1: 오류)")
    
    # Output Argument 2: ResultMessage (String)
    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.ValueRank = -1
    output_arg_2.Description = ua.LocalizedText("처리 상세 메시지")

    return [input_arg], [output_arg_1, output_arg_2]

def define_arm_001_arguments():
    input_arg = ua.Argument()
    input_arg.Name = "json_img_data_str"
    input_arg.DataType = ua.NodeId(ua.ObjectIds.String)
    input_arg.Description = ua.LocalizedText("Base64 인코딩된 이미지 데이터를 포함하는 JSON 문자열")

    output_arg_1 = ua.Argument()
    output_arg_1.Name = "ResultCode"
    output_arg_1.DataType = ua.NodeId(ua.ObjectIds.Int32)
    output_arg_1.Description = ua.LocalizedText("HTTP POST 결과 코드 (0: 성공, 1: 오류)")

    output_arg_2 = ua.Argument()
    output_arg_2.Name = "ResultMessage"
    output_arg_2.DataType = ua.NodeId(ua.ObjectIds.String)
    output_arg_2.Description = ua.LocalizedText("HTTP POST 상세 메시지")
    return [input_arg], [output_arg_1, output_arg_2]

# --- Modbus TCP Server 시작 함수 ---
def start_modbus_server():
    """Modbus TCP 서버를 별도 스레드에서 시작"""
    try:
        # Modbus 서버는 기본 포트 502를 사용합니다.
        StartTcpServer(context=modbus_context, host='192.168.1.2', port=502)
        print("Modbus TCP Server Started on 192.168.1.2:502")
    except Exception as e:
        print(f"Modbus TCP Server Failed to Start: {e}")

async def main():
    # -----------------------------------------------------
    # 1. OPC UA Server 설정
    # -----------------------------------------------------
    server = Server()
    await server.init()

    server.set_endpoint("opc.tcp://0.0.0.0:4840/freeopcua/server/")
    server.set_server_name("SynchroBots_OPCUA Server")

    server.set_security_policy([
        ua.SecurityPolicyType.NoSecurity
    ])

    uri = "http://SynchroBots.com/interfaces"
    idx = await server.register_namespace(uri)
    
    # Method 구현 클래스 초기화
    methods = ServerMethods(server, idx)
    
    # 2. Variable 노드 생성 및 Method 등록 Object 가져오기
    my_objects = await methods.init_nodes()


    # -----------------------------------------------------
    # 2. AMR_001 (amr_go_move) 메소드 정의 및 등록
    # -----------------------------------------------------
    amr_001_in_args, amr_001_out_args = define_amr_001_arguments()
    
    # 'amr_go_move' Method를 서버에 등록하고, 호출 시 call_amr_go_move 함수와 연결
    await my_objects.add_method(
        idx, 
        'amr_go_move', 
        methods.call_amr_go_move, 
        amr_001_in_args, 
        amr_001_out_args
    )

    # -----------------------------------------------------
    # 3. AMR_002 (amr_go_positions) 메소드 정의 및 등록 
    # -----------------------------------------------------
    amr_002_in_args, amr_002_out_args = define_amr_002_arguments()
    
    await my_objects.add_method(
        idx, 
        'amr_go_positions', 
        methods.call_amr_go_positions, 
        amr_002_in_args, 
        amr_002_out_args
    )

    # -----------------------------------------------------
    # 4. AMR_003 (amr_mission_state) 메소드 정의 및 등록
    # -----------------------------------------------------
    amr_003_in_args, amr_003_out_args = define_amr_003_arguments()
    
    # 'amr_mission_state' Method를 서버에 등록하고, 호출 시 call_amr_mission_state 함수와 연결
    await my_objects.add_method(
        idx, 
        'amr_mission_state', 
        methods.call_amr_mission_state, 
        amr_003_in_args, 
        amr_003_out_args
    )

    # -----------------------------------------------------
    # 5. PLC_001 (set_conveyorSensor_check) 메소드 정의 및 등록
    # -----------------------------------------------------
    plc_001_in_args, plc_001_out_args = define_plc_001_arguments()

    await my_objects.add_method(
        idx,
        'set_conveyorSensor_check',
        methods.call_set_conveyorSensor_check,
        plc_001_in_args,
        plc_001_out_args
    )
    
    # -----------------------------------------------------
    # 5. PLC_002 (OK_NG_Value) 메소드 정의 및 등록
    # -----------------------------------------------------
    plc_002_in_args, plc_002_out_args = define_plc_002_arguments()
    
    # 'OK_NG_Value' Method를 서버에 등록하고, 호출 시 call_ok_ng_value 함수와 연결
    await my_objects.add_method(
        idx, 
        'OK_NG_Value', 
        methods.call_ok_ng_value, 
        plc_002_in_args, 
        plc_002_out_args
    )

    # -----------------------------------------------------
    # 6. PLC_003 (set_robotArmSensor_check) 메소드 정의 및 등록
    # -----------------------------------------------------
    plc_003_in_args, plc_003_out_args = define_plc_003_arguments()
    
    # 'set_robotArmSensor_check' Method를 서버에 등록하고, 호출 시 call_set_robotArmSensor_check 함수와 연결
    await my_objects.add_method(
        idx, 
        'set_robotArmSensor_check', 
        methods.call_set_robotArmSensor_check, 
        plc_003_in_args, 
        plc_003_out_args
    )

    # -----------------------------------------------------
    # 7. PLC_004 (Ready_State) 메소드 정의 및 등록
    # -----------------------------------------------------
    plc_004_in_args, plc_004_out_args = define_plc_004_arguments()
    
    await my_objects.add_method(
        idx, 
        'Ready_State', 
        methods.call_ready_state, 
        plc_004_in_args, 
        plc_004_out_args
    )
    
    # -----------------------------------------------------
    # 8. ARM_001 (send_arm_img) 메소드 정의 및 등록
    # -----------------------------------------------------
    arm_001_in_args, arm_001_out_args = define_arm_001_arguments()

    await my_objects.add_method(
        idx,
        'send_arm_img',
        methods.call_send_arm_img,
        arm_001_in_args,
        arm_001_out_args
    )
    
    # -----------------------------------------------------
    # 9. 서버 실행
    # -----------------------------------------------------

    async with server:
        # 서버를 영원히 실행합니다.
        await asyncio.get_running_loop().create_future() 


if __name__ == "__main__":
    try:
         # 1) Modbus 서버는 별도 스레드에서 실행
        threading.Thread(target=start_modbus_server, daemon=True).start()
        # 2) OPC UA 서버 실행
        asyncio.run(main())
    except KeyboardInterrupt:
        # 서버 종료 로깅 (제거됨)
        pass # KeyboardInterrupt 발생 시 깔끔하게 종료
    except Exception as e:
        # 오류 발생 로깅 (제거됨)
        pass # 예외 발생 시 처리 로직