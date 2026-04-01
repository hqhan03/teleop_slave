#include "TeleopMasterClient.hpp"
#include "ManusSDKTypes.h"
#include <iostream>
#include <thread>
#include <cmath>
#include <array>
#include "ClientLogging.hpp"

using ManusSDK::ClientLog;

namespace {

std::array<float, 3> SubtractVec3(const ManusVec3& a, const ManusVec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

std::array<float, 3> RotateVec3ByInverseQuaternion(
    const std::array<float, 3>& v, const ManusQuaternion& q) {
    const float norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (norm_sq <= 1e-8f) {
        return v;
    }

    const float iw = q.w / norm_sq;
    const float ix = -q.x / norm_sq;
    const float iy = -q.y / norm_sq;
    const float iz = -q.z / norm_sq;

    const float t0w = -ix * v[0] - iy * v[1] - iz * v[2];
    const float t0x =  iw * v[0] + iy * v[2] - iz * v[1];
    const float t0y =  iw * v[1] + iz * v[0] - ix * v[2];
    const float t0z =  iw * v[2] + ix * v[1] - iy * v[0];

    return {
        -t0w * ix + t0x * iw - t0y * iz + t0z * iy,
        -t0w * iy + t0y * iw - t0z * ix + t0x * iz,
        -t0w * iz + t0z * iw - t0x * iy + t0y * ix,
    };
}

}  // namespace

// 싱글톤(Singleton) 인스턴스 전역 포인터 (콜백 함수에서 접근하기 위함)
TeleopMasterClient* TeleopMasterClient::s_Instance = nullptr;

int main(int argc, char* argv[]) {
    printf("=== TeleopMasterClient Starting ===\n"); fflush(stdout);

    TeleopMasterClient t_Client;

    // 1. 클라이언트(SDK 및 소켓) 초기화
    printf("[1/3] Initializing...\n"); fflush(stdout);
    ClientReturnCode initResult = t_Client.Initialize();
    if (initResult != ClientReturnCode::ClientReturnCode_Success) {
        printf("ERROR: Initialization failed (code: %d)\n", (int)initResult); fflush(stdout);
        printf("Make sure Manus Core is running.\n"); fflush(stdout);
        system("pause");
        return -1;
    }

    // 2. 메인 루프 실행 (데이터 수신 및 UDP 전송)
    printf("[2/3] Initialization OK. Starting main loop...\n"); fflush(stdout);
    t_Client.Run();

    // 3. 종료 시 리소스 정리
    printf("[3/3] Shutting down...\n"); fflush(stdout);
    t_Client.ShutDown();
    return 0;
}

TeleopMasterClient::TeleopMasterClient() : m_Running(true), m_ConnectionType(ConnectionType::ConnectionType_Local) {
    s_Instance = this;
    TrackerData_Init(&m_WristTracker);
    ErgonomicsData_Init(&m_RightGloveData);
}

TeleopMasterClient::~TeleopMasterClient() { s_Instance = nullptr; }


// UDP 소켓을 초기화하고 목적지(수신 서버)의 IP 및 포트를 설정
bool TeleopMasterClient::InitializeUDP(const char* ip, int port) {
    WSADATA wsaData;
    // Winsock 초기화
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) return false;
    
    // UDP 데이터그램 소켓 생성
    m_Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_Socket == INVALID_SOCKET) return false;

    // 목적지 주소 및 포트 바인딩 설정
    m_DestAddr.sin_family = AF_INET;
    m_DestAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &m_DestAddr.sin_addr);
    
    m_UdpInitialized = true;
    return true;
}


// 수집된 Hand/Wrist 데이터를 UDP 패킷 형태로 직렬화하여 송신
void TeleopMasterClient::SendUDPData(float posX, float posY, float posZ, float rotW, float rotX, float rotY, float rotZ) {
    if (!m_UdpInitialized) return;
    
    HandDataPacket packet;
    packet.frame = m_FrameCounter;

    // 1. 손목 트래커 위치 데이터 (X, Y, 기호 Z)
    packet.wristPos[0] = posX;
    packet.wristPos[1] = posY;
    packet.wristPos[2] = posZ;

    // 2. 손목 트래커 회전 데이터 (W, X, Y, Z 쿼터니언)
    packet.wristQuaternion[0] = rotW;
    packet.wristQuaternion[1] = rotX;
    packet.wristQuaternion[2] = rotY;
    packet.wristQuaternion[3] = rotZ;

    // 3. 우측 글로브 손가락 관절 데이터 추출 (오프셋 20부터 각 손가락당 4개의 관절 데이터 존재)
    int offset = 20;
    for (int i = 0; i < 5; i++) {
        // 엄지부터 새끼손가락까지 (Spread/CMC, Stretch/MCP, PIP, DIP) 순서로 구조체에 복사
        packet.fingerFlexion[i * 4 + 0] = m_RightGloveData.data[offset + (i * 4) + 0];
        packet.fingerFlexion[i * 4 + 1] = m_RightGloveData.data[offset + (i * 4) + 1];
        packet.fingerFlexion[i * 4 + 2] = m_RightGloveData.data[offset + (i * 4) + 2];
        packet.fingerFlexion[i * 4 + 3] = m_RightGloveData.data[offset + (i * 4) + 3];
    }

    // 4. 손끝(Fingertip) 위치 데이터 복사 (5 fingers × XYZ)
    for (int i = 0; i < 15; i++) {
        packet.fingertipPos[i] = m_FingertipPositions[i];
    }

    // 5. 전체 25개 랜드마크 위치 데이터 복사 (5 fingers × 5 joints × XYZ)
    for (int i = 0; i < 75; i++) {
        packet.landmarkPos[i] = m_LandmarkPositions[i];
    }

    // UDP 데이터 전송
    sendto(m_Socket, (char*)&packet, sizeof(packet), 0, (sockaddr*)&m_DestAddr, sizeof(m_DestAddr));
}

ClientReturnCode TeleopMasterClient::Initialize() {
    printf("  - PlatformSpecificInitialization()...\n"); fflush(stdout);
    if (!PlatformSpecificInitialization()) {
        printf("  ERROR: PlatformSpecificInitialization() failed!\n"); fflush(stdout);
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }
    printf("  - PlatformSpecificInitialization() OK.\n"); fflush(stdout);
    return InitializeSDK();
}

ClientReturnCode TeleopMasterClient::InitializeSDK() {
    printf("  - CoreSdk_InitializeCore()...\n"); fflush(stdout);
    SDKReturnCode t_SdkResult = CoreSdk_InitializeCore();
    if (t_SdkResult != SDKReturnCode::SDKReturnCode_Success) {
        printf("  ERROR: CoreSdk_InitializeCore() failed (SDK code: %d)\n", (int)t_SdkResult); fflush(stdout);
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    }
    printf("  - CoreSdk_InitializeCore() OK.\n"); fflush(stdout);
    printf("  - Registering callbacks...\n"); fflush(stdout);
    RegisterAllCallbacks();
    printf("  - Setting coordinate system...\n"); fflush(stdout);
    CoordinateSystemVUH t_VUH = { AxisView::AxisView_XFromViewer, AxisPolarity::AxisPolarity_PositiveZ, Side::Side_Right, 1.0f };
    CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, true);
    printf("  - SDK initialization complete.\n"); fflush(stdout);
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode TeleopMasterClient::RegisterAllCallbacks() {
    CoreSdk_RegisterCallbackForTrackerStream(*OnTrackerStreamCallback);
    CoreSdk_RegisterCallbackForErgonomicsStream(*OnErgonomicsCallback);
    CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback);
    CoreSdk_RegisterCallbackForRawSkeletonStream(*OnRawSkeletonStreamCallback);
    return ClientReturnCode::ClientReturnCode_Success;
}

// [Landscape 콜백] 연결된 기기들의 환경(Landscape) 정보를 받아와 우측 글로브의 ID를 획득
void TeleopMasterClient::OnLandscapeCallback(const Landscape* const p_Landscape) {
    if (!s_Instance) return;
    std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
    for (uint32_t i = 0; i < p_Landscape->gloveDevices.gloveCount; i++) {
        if (p_Landscape->gloveDevices.gloves[i].side == Side_Right)
            s_Instance->m_RightGloveID = p_Landscape->gloveDevices.gloves[i].id;
    }
}

// [Ergonomics 콜백] 인체공학적(Ergonomics) 관절 각도 데이터를 실시간으로 수신
void TeleopMasterClient::OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo) {
    if (!s_Instance) return;
    std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
    for (uint32_t i = 0; i < p_Ergo->dataCount; i++) {
        if (p_Ergo->data[i].id == s_Instance->m_RightGloveID) s_Instance->m_RightGloveData = p_Ergo->data[i];
    }
}

// [Raw Skeleton 콜백] 원시 골격 데이터를 수신하여 손끝(Fingertip) 위치를 추출
void TeleopMasterClient::OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_Info) {
    if (!s_Instance || !p_Info || p_Info->skeletonsCount == 0) return;
    std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);

    if (s_Instance->m_RightGloveID == 0) return;

    // 스트림에서 우측 글로브에 해당하는 원시 골격을 찾기
    for (uint32_t i = 0; i < p_Info->skeletonsCount; i++) {
        RawSkeletonInfo t_Info;
        if (CoreSdk_GetRawSkeletonInfo(i, &t_Info) != SDKReturnCode::SDKReturnCode_Success) continue;
        if (t_Info.gloveId != s_Instance->m_RightGloveID) continue;
        if (t_Info.nodesCount == 0) continue;

        // Tip/Wrist 노드를 한 번만 확인 (최초 프레임에서 매핑 수행)
        if (!s_Instance->m_RawSkeletonNodesResolved) {
            uint32_t nodeCount = 0;
            if (CoreSdk_GetRawSkeletonNodeCount(s_Instance->m_RightGloveID, nodeCount) != SDKReturnCode::SDKReturnCode_Success) break;
            if (nodeCount == 0) break;

            std::vector<NodeInfo> nodeInfos(nodeCount);
            if (CoreSdk_GetRawSkeletonNodeInfoArray(s_Instance->m_RightGloveID, nodeInfos.data(), nodeCount) != SDKReturnCode::SDKReturnCode_Success) break;

            // Diagnostic: dump all node metadata once to identify correct node types
            static bool s_DumpedOnce = false;
            if (!s_DumpedOnce) {
                s_DumpedOnce = true;
                printf("\n=== RAW SKELETON NODE DUMP (nodeCount=%u) ===\n", nodeCount);
                for (uint32_t n = 0; n < nodeCount; n++) {
                    printf("  Node[%u]: id=%u parent=%u chainType=%d side=%d fingerJointType=%d\n",
                        n, nodeInfos[n].nodeId, nodeInfos[n].parentId,
                        (int)nodeInfos[n].chainType, (int)nodeInfos[n].side,
                        (int)nodeInfos[n].fingerJointType);
                }
                printf("=== END NODE DUMP ===\n\n");
                fflush(stdout);
                s_Instance->m_NodeDumpTime = std::chrono::steady_clock::now();
            }

            // ChainType/FingerJointType → raw node ID 매핑
            // Resolve all 25 landmark nodes (5 fingers × 5 joint types) + wrist
            for (uint32_t n = 0; n < nodeCount; n++) {
                if (nodeInfos[n].chainType == ChainType_Hand) {
                    s_Instance->m_WristNodeId = nodeInfos[n].nodeId;
                    s_Instance->m_WristNodeResolved = true;
                    continue;
                }

                // Map ChainType to finger index
                int fingerIdx = -1;
                switch (nodeInfos[n].chainType) {
                case ChainType_FingerThumb:  fingerIdx = 0; break;
                case ChainType_FingerIndex:  fingerIdx = 1; break;
                case ChainType_FingerMiddle: fingerIdx = 2; break;
                case ChainType_FingerRing:   fingerIdx = 3; break;
                case ChainType_FingerPinky:  fingerIdx = 4; break;
                default: continue;
                }

                // Map FingerJointType to joint index within finger
                int jointIdx = -1;
                switch (nodeInfos[n].fingerJointType) {
                case FingerJointType_Metacarpal:   jointIdx = 0; break;
                case FingerJointType_Proximal:     jointIdx = 1; break;
                case FingerJointType_Intermediate: jointIdx = 2; break;
                case FingerJointType_Distal:       jointIdx = 3; break;
                case FingerJointType_Tip:          jointIdx = 4; break;
                default: continue;
                }

                // Store in 25-landmark array: finger * 5 + joint
                int landmarkIdx = fingerIdx * 5 + jointIdx;
                s_Instance->m_LandmarkNodeIds[landmarkIdx] = nodeInfos[n].nodeId;

                // Also keep existing tip node IDs for backward compat
                if (jointIdx == 4) {
                    s_Instance->m_TipNodeIds[fingerIdx] = nodeInfos[n].nodeId;
                }
            }

            // Check that wrist + all 5 tip nodes are resolved (minimum for operation)
            bool allTipNodesResolved = s_Instance->m_WristNodeResolved;
            for (int f = 0; f < 5; ++f) {
                if (s_Instance->m_TipNodeIds[f] == 0) {
                    allTipNodesResolved = false;
                    break;
                }
            }

            if (!allTipNodesResolved) {
                ClientLog::warn("Failed to resolve MANUS raw skeleton wrist/tip nodes yet.");
                break;
            }

            s_Instance->m_RawSkeletonNodesResolved = true;

            // Check how many of the 25 landmark nodes were resolved
            int landmarkCount = 0;
            for (int lm = 0; lm < 25; lm++) {
                if (s_Instance->m_LandmarkNodeIds[lm] != 0) landmarkCount++;
            }
            s_Instance->m_LandmarkNodesResolved = (landmarkCount > 0);
            printf("[Landmarks] Resolved %d/25 landmark nodes.\n", landmarkCount);
            fflush(stdout);
        }

        if (!s_Instance->m_RawSkeletonNodesResolved) {
            break;
        }

        // 골격 노드 데이터를 가져와서 wrist-local Tip 위치 추출
        std::vector<SkeletonNode> nodes(t_Info.nodesCount);
        if (CoreSdk_GetRawSkeletonData(i, nodes.data(), t_Info.nodesCount) != SDKReturnCode::SDKReturnCode_Success) break;

        const SkeletonNode* wristNode = nullptr;
        for (const auto& node : nodes) {
            if (node.id == s_Instance->m_WristNodeId) {
                wristNode = &node;
                break;
            }
        }
        if (wristNode == nullptr) {
            ClientLog::warn("MANUS raw skeleton frame missing wrist node.");
            break;
        }

        // Extract wrist-local positions for all 25 landmarks
        for (int lm = 0; lm < 25; lm++) {
            if (s_Instance->m_LandmarkNodeIds[lm] == 0) continue;
            const SkeletonNode* lmNode = nullptr;
            for (const auto& node : nodes) {
                if (node.id == s_Instance->m_LandmarkNodeIds[lm]) {
                    lmNode = &node;
                    break;
                }
            }
            if (lmNode != nullptr) {
                const auto pos_from_wrist =
                    SubtractVec3(lmNode->transform.position, wristNode->transform.position);
                const auto pos_wrist_local =
                    RotateVec3ByInverseQuaternion(pos_from_wrist, wristNode->transform.rotation);

                s_Instance->m_LandmarkPositions[lm * 3 + 0] = pos_wrist_local[0];
                s_Instance->m_LandmarkPositions[lm * 3 + 1] = pos_wrist_local[1];
                s_Instance->m_LandmarkPositions[lm * 3 + 2] = pos_wrist_local[2];
            }
        }

        // Copy tip landmarks into legacy fingertip array for backward compat
        for (int f = 0; f < 5; f++) {
            int tipLandmarkIdx = f * 5 + 4; // Tip is joint index 4
            s_Instance->m_FingertipPositions[f * 3 + 0] = s_Instance->m_LandmarkPositions[tipLandmarkIdx * 3 + 0];
            s_Instance->m_FingertipPositions[f * 3 + 1] = s_Instance->m_LandmarkPositions[tipLandmarkIdx * 3 + 1];
            s_Instance->m_FingertipPositions[f * 3 + 2] = s_Instance->m_LandmarkPositions[tipLandmarkIdx * 3 + 2];
        }

        break; // 우측 글로브 골격을 찾았으므로 루프 종료
    }
}

// [Tracker 콜백] VIVE 트래커 등에서 발생하는 위치 및 회전 데이터를 실시간으로 수신
void TeleopMasterClient::OnTrackerStreamCallback(const TrackerStreamInfo* const p_Info) {
    if (s_Instance && p_Info->trackerCount > 0) {
        std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
        bool trackerFound = false;
        TrackerData t_TrackerData;
        
        // 연결된 트래커 중 우측 손(RightHand) 트래커를 찾아 데이터를 저장
        for (uint32_t i = 0; i < p_Info->trackerCount; i++) {
            if (CoreSdk_GetTrackerData(i, &t_TrackerData) == SDKReturnCode::SDKReturnCode_Success) {
                if (t_TrackerData.trackerType == TrackerType::TrackerType_RightHand) {
                    s_Instance->m_WristTracker = t_TrackerData;
                    trackerFound = true;
                    break;
                }
            }
        }
        
        // 명시적인 RightHand 트래커가 없을 경우 첫 번째 트래커를 사용 (Fallback)
        if (!trackerFound && CoreSdk_GetTrackerData(0, &t_TrackerData) == SDKReturnCode::SDKReturnCode_Success) {
            s_Instance->m_WristTracker = t_TrackerData;
        }
    }
}

// 메인 동작 루프: UDP 설정, Manus 호스트 연결, 실시간 데이터 송신 및 콘솔 출력 수행
void TeleopMasterClient::Run() {
    printf("[Run] Initializing UDP...\n"); fflush(stdout);

    // [중요] 타겟 수신 PC (예: ROS2가 실행 중인 Ubuntu)의 실제 IP 및 Port로 변경하세요.
    if (!InitializeUDP("192.168.0.112", 12345)) {
        printf("[Run] ERROR: UDP init failed!\n"); fflush(stdout);
        ClientLog::error("Failed to initialize UDP.");
        system("pause");
        return;
    }
    printf("[Run] UDP OK. Connecting to Manus Core...\n"); fflush(stdout);

    // Manus Core와 연결될 때까지 주기적으로 재시도 (1초 간격)
    while (Connect() != ClientReturnCode::ClientReturnCode_Success) {
        printf("[Run] Connection failed, retrying in 1s...\n"); fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    printf("[Run] Connected! Setting raw skeleton hand motion...\n"); fflush(stdout);
    CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);
    printf("[Run] Entering main loop.\n"); fflush(stdout);

    // 메인 데이터 스트리밍 루프 시작
    while (m_Running) {
        {
            // 백그라운드 콜백에서 데이터가 업데이트되는 것을 보호하기 위해 mutex 잠금
            std::lock_guard<std::mutex> lock(m_DataMutex);
            
            // 캘리브레이션 적용된 위치 및 회전 계산
            float calibPosX = m_WristTracker.position.x - m_PosOffsetX;
            float calibPosY = m_WristTracker.position.y - m_PosOffsetY;
            float calibPosZ = m_WristTracker.position.z - m_PosOffsetZ;

            // 회전 적용 (q_new = q_offset * q_raw)
            float rw = m_WristTracker.rotation.w;
            float rx = m_WristTracker.rotation.x;
            float ry = m_WristTracker.rotation.y;
            float rz = m_WristTracker.rotation.z;

            float calibRotW = m_RotOffsetW * rw - m_RotOffsetX * rx - m_RotOffsetY * ry - m_RotOffsetZ * rz;
            float calibRotX = m_RotOffsetW * rx + m_RotOffsetX * rw + m_RotOffsetY * rz - m_RotOffsetZ * ry;
            float calibRotY = m_RotOffsetW * ry - m_RotOffsetX * rz + m_RotOffsetY * rw + m_RotOffsetZ * rx;
            float calibRotZ = m_RotOffsetW * rz + m_RotOffsetX * ry - m_RotOffsetY * rx + m_RotOffsetZ * rw;

            // 현재 프레임의 데이터를 타겟 IP로 전송
            SendUDPData(calibPosX, calibPosY, calibPosZ, calibRotW, calibRotX, calibRotY, calibRotZ);

            // Skip cls for 10s after node dump to let user read diagnostic output
            auto timeSinceDump = std::chrono::steady_clock::now() - m_NodeDumpTime;
            if (m_NodeDumpTime == std::chrono::steady_clock::time_point{} ||
                timeSinceDump > std::chrono::seconds(10)) {
                system("cls");
            } else {
                printf("\n--- Node dump displayed above (%.0fs remaining) ---\n",
                    10.0 - std::chrono::duration<double>(timeSinceDump).count());
            }
            printf("=== MANUS Core -> ROS2 Humble (UDP 50Hz) ===\n");
            printf("[VIVE Tracker] Pos: X:%.3f Y:%.3f Z:%.3f | Quat: W:%.3f X:%.3f Y:%.3f Z:%.3f\n",
                calibPosX, calibPosY, calibPosZ, calibRotW, calibRotX, calibRotY, calibRotZ);
            
            if (m_Calibrated) {
                printf("[!] Tracker Zeroed (Calibration Applied).\n");
            }

            if (m_RightGloveID != 0) {
                printf("[MANUS Glove ID: 0x%X] Sending 20 Finger Joints...\n", m_RightGloveID);
                printf("  Thumb:  MCPSpread=%.2f MCPStrech=%.2f PIPStrech=%.2f DIPStrech=%.2f\n",
                    m_RightGloveData.data[20], m_RightGloveData.data[21], m_RightGloveData.data[22], m_RightGloveData.data[23]);
                printf("  Index:  MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[24], m_RightGloveData.data[25], m_RightGloveData.data[26], m_RightGloveData.data[27]);
                printf("  Middle: MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[28], m_RightGloveData.data[29], m_RightGloveData.data[30], m_RightGloveData.data[31]);
                printf("  Ring:   MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[32], m_RightGloveData.data[33], m_RightGloveData.data[34], m_RightGloveData.data[35]);
                printf("  Pinky:  MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[36], m_RightGloveData.data[37], m_RightGloveData.data[38], m_RightGloveData.data[39]);

                if (m_RawSkeletonNodesResolved) {
                    const char* fingerNames[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
                    printf("[Fingertip Positions (Raw Skeleton Wrist-Local Frame)]\n");
                    for (int f = 0; f < 5; f++) {
                        printf("  %-7s: X:%.4f Y:%.4f Z:%.4f\n", fingerNames[f],
                            m_FingertipPositions[f * 3 + 0], m_FingertipPositions[f * 3 + 1], m_FingertipPositions[f * 3 + 2]);
                    }
                } else {
                    printf("[Fingertip Positions] Waiting for raw skeleton node resolution...\n");
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // Spacebar 입력 시 현재 위치 및 회전을 0으로 설정(캘리브레이션)
        if (GetKeyDown(' ')) {
            std::lock_guard<std::mutex> lock(m_DataMutex);
            m_PosOffsetX = m_WristTracker.position.x;
            m_PosOffsetY = m_WristTracker.position.y;
            m_PosOffsetZ = m_WristTracker.position.z;

            // 회전 역산(켤레 복소수) 저장
            m_RotOffsetW = m_WristTracker.rotation.w;
            m_RotOffsetX = -m_WristTracker.rotation.x;
            m_RotOffsetY = -m_WristTracker.rotation.y;
            m_RotOffsetZ = -m_WristTracker.rotation.z;
            m_Calibrated = true;
        }

        // ESC 입력 시 종료
        if (GetKeyDown(VK_ESCAPE)) m_Running = false;

        m_FrameCounter++;
    }
}

ClientReturnCode TeleopMasterClient::Connect() {
    if (CoreSdk_LookForHosts(1, true) != SDKReturnCode::SDKReturnCode_Success) return ClientReturnCode::ClientReturnCode_FailedToConnect;
    uint32_t count = 0;
    CoreSdk_GetNumberOfAvailableHostsFound(&count);
    if (count == 0) return ClientReturnCode::ClientReturnCode_FailedToConnect;
    ManusHost host;
    CoreSdk_GetAvailableHostsFound(&host, 1);
    return (CoreSdk_ConnectToHost(host) == SDKReturnCode::SDKReturnCode_Success) ? ClientReturnCode::ClientReturnCode_Success : ClientReturnCode::ClientReturnCode_FailedToConnect;
}

ClientReturnCode TeleopMasterClient::ShutDown() {
    if (m_UdpInitialized) { closesocket(m_Socket); WSACleanup(); }
    CoreSdk_ShutDown();
    PlatformSpecificShutdown();
    return ClientReturnCode::ClientReturnCode_Success;
}


