#include "pch.h"
#include "CameraController.h"
#include "../Core/Logger.h"

namespace Camera {

    CameraController::CameraController() {}

    void CameraController::OnUpdateBuffer(reshade::api::resource resource, const void* data, uint64_t size) {
        ScanBufferImpl(resource, data, size, false);
    }

    void CameraController::OnScanBuffer(reshade::api::resource resource, const void* data, uint64_t size) {
        ScanBufferImpl(resource, data, size, true);
    }

    void CameraController::ScanBufferImpl(reshade::api::resource resource, const void* data, uint64_t size, bool isMapped) {
        if (size < 64) return; // Too small for a matrix

        // Performance Guard: If mapped (uncached memory), ONLY read small buffers
        if (isMapped && size > 4096) return;

        static int logCounter = 0;
        bool logThisFrame = (logCounter++ % 500 == 0); // Log occasionally to avoid spam

        std::lock_guard<std::mutex> lock(m_mutex);
        
        uint64_t handle = resource.handle;
        auto& state = m_bufferCache[handle];
        
        // Update cache
        if (state.data.size() != size) state.data.resize(size);
        memcpy(state.data.data(), data, size);

        const float* floatData = (const float*)data;
        size_t floatCount = size / sizeof(float);

        // Check if this is the "Noisy" buffer (Size ~10KB)
        bool isNoisy = (size > 9000 && size < 11000);
        
        static int noiseLogCount = 0;
        
        // Log this frame if:
        // 1. It's NOT the noisy buffer
        // 2. OR it IS the noisy buffer, but we haven't logged it much yet
        // 3. AND global log counter is not exhausted (reset counter for this logic)
        
        bool shouldLog = false;
        if (logThisFrame) {
            if (!isNoisy) {
                shouldLog = true;
            } else if (noiseLogCount < 5) {
                shouldLog = true;
                noiseLogCount++;
            }
        }

        bool foundView = false;
        bool foundProj = false;

        if (shouldLog && !foundView && !foundProj && m_cameraBuffer.handle == 0) {
            // Log first few floats of a candidate buffer if we haven't found a camera yet
            LOG_INFO("Scanning Buffer ", (void*)handle, " Size: ", size, " (Mapped: ", isMapped, "). F[0-3]: ", floatData[0], ", ", floatData[1], ", ", floatData[2], ", ", floatData[3]);
        }

        // FULL BUFFER DUMP (Only for medium/small buffers now, OR specifically requested)
        if (m_cameraBuffer.handle == 0 && !m_deepScanDone && size > 200 && size < 2000) { // Look for standard CB sizes!
             m_deepScanDone = true; 
             LOG_INFO("--- FULL BUFFER DUMP START [Buffer ", (void*)handle, " Size ", size, "] ---");
                 
             // Dump all floats in lines of 8
             for (size_t i = 0; i < floatCount; i += 8) {
                 if (i + 7 < floatCount) {
                     LOG_INFO("OFFSET ", i*4, ": ", 
                         floatData[i], ", ", floatData[i+1], ", ", floatData[i+2], ", ", floatData[i+3], ",    ",
                         floatData[i+4], ", ", floatData[i+5], ", ", floatData[i+6], ", ", floatData[i+7]);
                 }
             }
             LOG_INFO("--- FULL BUFFER DUMP END ---");
        }

        // Scan for View Matrix
        for (size_t i = 0; i <= floatCount - 16; i += 4) {
            bool transposed = false;
            // Original logic for view matrix
            if (IsViewMatrix(floatData + i, &transposed)) {
                 if (!foundView) {
                     // LOG_INFO("Found Potential View Matrix at Offset ", i*4);
                 }
                 state.viewMatrixOffset = (int)i;
                 state.isCamera = true;
                 
                 LOG_INFO("FOUND VIEW MATRIX! Buffer: ", (void*)handle, " Offset: ", i);

                 m_isTransposed = transposed;
                 DirectX::XMMATRIX viewMat = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4*)(floatData + i));
                 if (transposed) viewMat = DirectX::XMMatrixTranspose(viewMat);

                 m_lastGameView = viewMat;

                 if (!m_upDetected) {
                     DetectWorldUp(viewMat);
                 }

                 m_cameraBuffer = resource; 
                 foundView = true;
                 break; 
            }
        }

        // Scan for Projection Matrix
        for (size_t i = 0; i <= floatCount - 16; i += 4) {
            if (IsProjectionMatrix(floatData + i)) {
                state.projMatrixOffset = (int)i;
                state.isCamera = true;

                LOG_INFO("FOUND PROJ MATRIX! Buffer: ", (void*)handle, " Offset: ", i);

                m_isRH = IsRightHandedProjection(floatData + i);
                m_lastGameProj = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4*)(floatData + i));

                m_cameraBuffer = resource;
                foundProj = true;
                break;
            }
        }
    }

    bool CameraController::GetModifiedBufferData(CubeFace face, std::vector<uint8_t>& outputData) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_cameraBuffer.handle == 0) return false;

        auto it = m_bufferCache.find(m_cameraBuffer.handle);
        if (it == m_bufferCache.end()) return false;

        auto& state = it->second;
        outputData = state.data; // Copy original data

        float* outFloats = (float*)outputData.data();
        size_t floatCount = outputData.size() / sizeof(float);

        // Replace View Matrix
        if (state.viewMatrixOffset >= 0 && (size_t)(state.viewMatrixOffset + 16) <= floatCount) {
            DirectX::XMMATRIX newView = GetViewMatrixForFace(face);
            if (m_isTransposed) newView = DirectX::XMMatrixTranspose(newView);

            DirectX::XMStoreFloat4x4((DirectX::XMFLOAT4X4*)(outFloats + state.viewMatrixOffset), newView);
        }

        // Replace Projection Matrix (Force 90 degree FOV)
        if (state.projMatrixOffset >= 0 && (size_t)(state.projMatrixOffset + 16) <= floatCount) {
            DirectX::XMMATRIX newProj;
            if (m_isRH) {
                newProj = DirectX::XMMatrixPerspectiveFovRH(DirectX::XM_PIDIV2, 1.0f, 0.1f, 1000.0f);
            } else {
                newProj = DirectX::XMMatrixPerspectiveFovLH(DirectX::XM_PIDIV2, 1.0f, 0.1f, 1000.0f);
            }
            DirectX::XMStoreFloat4x4((DirectX::XMFLOAT4X4*)(outFloats + state.projMatrixOffset), newProj);
        }

        return true;
    }

    DirectX::XMMATRIX CameraController::GetViewMatrixForFace(CubeFace face) {
        DirectX::XMVECTOR det;
        DirectX::XMMATRIX invView = DirectX::XMMatrixInverse(&det, m_lastGameView);
        DirectX::XMVECTOR eyePos = invView.r[3];

        bool isZUp = (std::abs(DirectX::XMVectorGetZ(m_worldUp)) > 0.9f);

        DirectX::XMVECTOR vRight, vLeft, vUp, vDown, vFront, vBack;

        if (isZUp) {
             // Z-Up System
             vRight = DirectX::XMVectorSet(1, 0, 0, 0);
             vLeft  = DirectX::XMVectorSet(-1, 0, 0, 0);
             vUp    = DirectX::XMVectorSet(0, 0, 1, 0);
             vDown  = DirectX::XMVectorSet(0, 0, -1, 0);
             vFront = DirectX::XMVectorSet(0, 1, 0, 0);
             vBack  = DirectX::XMVectorSet(0, -1, 0, 0);
        } else {
             // Y-Up System
             vRight = DirectX::XMVectorSet(1, 0, 0, 0);
             vLeft  = DirectX::XMVectorSet(-1, 0, 0, 0);
             vUp    = DirectX::XMVectorSet(0, 1, 0, 0);
             vDown  = DirectX::XMVectorSet(0, -1, 0, 0);
             if (m_isRH) {
                 vFront = DirectX::XMVectorSet(0, 0, -1, 0);
                 vBack  = DirectX::XMVectorSet(0, 0, 1, 0);
             } else {
                 vFront = DirectX::XMVectorSet(0, 0, 1, 0);
                 vBack  = DirectX::XMVectorSet(0, 0, -1, 0);
             }
        }

        DirectX::XMVECTOR targetDir;
        DirectX::XMVECTOR upDir = m_worldUp;

        switch (face) {
            case CubeFace::Right: targetDir = vRight; break;
            case CubeFace::Left:  targetDir = vLeft; break;
            case CubeFace::Up:    targetDir = vUp; upDir = vFront; break;
            case CubeFace::Down:  targetDir = vDown; upDir = DirectX::XMVectorNegate(vFront); break;
            case CubeFace::Front: targetDir = vFront; break;
            case CubeFace::Back:  targetDir = vBack; break;
        }

        if (m_isRH) return DirectX::XMMatrixLookAtRH(eyePos, DirectX::XMVectorAdd(eyePos, targetDir), upDir);
        else        return DirectX::XMMatrixLookAtLH(eyePos, DirectX::XMVectorAdd(eyePos, targetDir), upDir);
    }

    bool CameraController::IsProjectionMatrix(const float* data) {
        const float epsilon = 0.1f;
        // Check for projection patterns (0s in specific spots)
        // [ x 0 0 0 ]
        // [ 0 x 0 0 ]
        // [ 0 0 x x ]
        // [ 0 0 x 0 ]
        if (std::abs(data[1]) > epsilon || std::abs(data[2]) > epsilon || std::abs(data[3]) > epsilon) return false;
        if (std::abs(data[4]) > epsilon || std::abs(data[6]) > epsilon || std::abs(data[7]) > epsilon) return false;
        if (std::abs(data[15]) > epsilon) return false;
        
        // Check w components
        if (std::abs(data[11] - 1.0f) > epsilon && std::abs(data[11] + 1.0f) > epsilon) return false;

        return true;
    }

    bool CameraController::IsViewMatrix(const float* data, bool* outIsTransposed) {
        const float epsilon = 0.1f;
        // Row Major: [x x x 0], [x x x 0], [x x x 0], [x x x 1]
        bool rowMajor = (std::abs(data[3]) < epsilon && std::abs(data[7]) < epsilon && std::abs(data[11]) < epsilon && std::abs(data[15] - 1.0f) < epsilon);
        // Col Major (Transposed): [x x x x], [x x x x], [x x x x], [0 0 0 1]
        bool colMajor = (std::abs(data[12]) < epsilon && std::abs(data[13]) < epsilon && std::abs(data[14]) < epsilon && std::abs(data[15] - 1.0f) < epsilon);

        if (outIsTransposed) *outIsTransposed = colMajor;

        // To distinguish from World Matrix, we check if it is orthogonal (rotation part).
        // View Matrix rotation part is orthogonal. World Matrix can be scaled.
        // For now, heuristic is weak but matches original code intent.

        return rowMajor || colMajor;
    }

    bool CameraController::IsRightHandedProjection(const float* data) {
        // [2][3] (index 11) is -1 for RH, 1 for LH usually
        return (data[11] < -0.9f);
    }

    void CameraController::DetectWorldUp(DirectX::XMMATRIX viewMat) {
        DirectX::XMVECTOR det;
        DirectX::XMMATRIX invView = DirectX::XMMatrixInverse(&det, viewMat);
        DirectX::XMVECTOR up = invView.r[1];
        up = DirectX::XMVector3Normalize(up);

        float y = DirectX::XMVectorGetY(up);
        float z = DirectX::XMVectorGetZ(up);

        if (std::abs(z) > std::abs(y)) {
            if (z > 0) m_worldUp = DirectX::XMVectorSet(0, 0, 1, 0);
            else       m_worldUp = DirectX::XMVectorSet(0, 0, -1, 0);
            LOG_INFO("Detected Z-Up World");
        } else {
            if (y > 0) m_worldUp = DirectX::XMVectorSet(0, 1, 0, 0);
            else       m_worldUp = DirectX::XMVectorSet(0, -1, 0, 0);
            LOG_INFO("Detected Y-Up World");
        }
        m_upDetected = true;
    }
}
