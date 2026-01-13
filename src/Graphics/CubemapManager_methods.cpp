
    void CubemapManager::OnMapBuffer(reshade::api::device* /*device*/, reshade::api::resource resource, uint64_t size, void* data) {
         if (!data) return;
         std::lock_guard<std::mutex> lock(m_mapMutex);
         m_mappedBuffers[resource.handle] = { data, size };
    }

    void CubemapManager::OnUnmapBuffer(reshade::api::device* /*device*/, reshade::api::resource resource) {
        // Read data on unmap
        void* dataPtr = nullptr;
        uint64_t size = 0;
        
        {
            std::lock_guard<std::mutex> lock(m_mapMutex);
            auto it = m_mappedBuffers.find(resource.handle);
            if (it != m_mappedBuffers.end()) {
                dataPtr = it->second.data;
                size = it->second.size;
                m_mappedBuffers.erase(it);
            }
        }

        if (dataPtr && size > 64) {
             if (m_cameraController) {
                 m_cameraController->OnUpdateBuffer(resource, dataPtr, size);
             }
        }
    }
