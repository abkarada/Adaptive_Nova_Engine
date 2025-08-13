#include "gpu_detect.hpp"
#include <cstdlib>
#include <fstream>
#include <string>

bool has_nvidia_gpu() {
    std::ifstream f("/proc/driver/nvidia/version");
    return f.good();
}

bool has_vaapi_gpu() {
    std::ifstream f("/dev/dri/renderD128");
    return f.good();
}

bool has_qsv_gpu() {
    std::ifstream f("/dev/dri/renderD128");
    if (!f.good()) return false;
    
    // Intel GPU varlığını kontrol et
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line)) {
        if (line.find("vendor_id") != std::string::npos && 
            line.find("GenuineIntel") != std::string::npos) {
            return true;
        }
    }
    return false;
}