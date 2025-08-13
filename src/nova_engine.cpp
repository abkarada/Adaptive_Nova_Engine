#include "common.hpp"
#include "gpu_detect.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>
#include <optional>
#include <climits>
#include <tuple>
#include <mutex>
#include <map>
#include <cmath>
#include <algorithm>
#include <memory>

// Linux UDP (kontrol kanalı - raw)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// GLib/GIO
#include <gio/gio.h>
#include <gst/app/gstappsrc.h>

// Reed-Solomon
#include <isa-l/erasure_code.h>

// Zaman yardımcıları
static inline uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

// RTP sabitler
static constexpr uint16_t RTP_HEADER_SIZE = 12;
static constexpr uint32_t RTP_CLOCK_RATE = 90000;

// Zero-copy buffer flag
#ifndef GST_BUFFER_FLAG_ZERO_COPY
#define GST_BUFFER_FLAG_ZERO_COPY (GST_BUFFER_FLAG_LAST << 1)
#endif

#pragma pack(push, 1)
// Chunk yapıları
struct ChunkHeader {
    uint32_t magic = 0xABCD1234;
    uint32_t frame_id = 0;        // Video frame ID
    uint16_t chunk_index = 0;     // Chunk sıra no
    uint16_t total_chunks = 0;    // Frame'deki toplam chunk
    uint16_t k_data = 0;          // FEC data chunk sayısı
    uint16_t r_parity = 0;        // FEC parity chunk sayısı
    uint16_t payload_bytes = 0;   // Chunk payload boyutu
    uint32_t total_frame_bytes = 0;// Frame toplam boyutu
    uint32_t rtp_timestamp = 0;   // RTP ile senkronize timestamp
    uint16_t rtp_seq = 0;         // RTP sequence number
    uint8_t  flags = 0;           // bit0: parity, bit1: keyframe
    uint32_t checksum = 0;        // FNV-1a checksum
};
#pragma pack(pop)

// UDP profil metrikleri
struct UDPStats {
    double packet_loss = 0.0;    // paket kayıp oranı
    double avg_rtt_ms = 0.0;     // ortalama RTT
    double jitter_ms = 0.0;      // jitter (RTT varyansı)
    double burst_loss = 0.0;     // ardışık kayıp oranı
    double bandwidth_mbps = 0.0; // tahmini bant genişliği
    
    std::vector<double> rtt_samples;  // RTT örnekleri (jitter için)
    std::vector<bool> loss_pattern;   // kayıp paterni (burst için)
    static constexpr size_t MAX_SAMPLES = 100;
    
    void update_rtt(double rtt_ms) {
        // EWMA ile RTT güncelle
        avg_rtt_ms = (avg_rtt_ms > 0) ? (avg_rtt_ms * 0.8 + rtt_ms * 0.2) : rtt_ms;
        
        // Jitter hesapla
        rtt_samples.push_back(rtt_ms);
        if (rtt_samples.size() > MAX_SAMPLES) rtt_samples.erase(rtt_samples.begin());
        if (rtt_samples.size() >= 2) {
            double sum = 0, mean = avg_rtt_ms;
            for (double s : rtt_samples) sum += (s - mean) * (s - mean);
            jitter_ms = std::sqrt(sum / rtt_samples.size());
        }
    }
    
    void update_loss(bool is_lost) {
        // Kayıp paterni güncelle
        loss_pattern.push_back(is_lost);
        if (loss_pattern.size() > MAX_SAMPLES) loss_pattern.erase(loss_pattern.begin());
        
        // Genel kayıp oranı
        size_t lost = 0;
        for (bool l : loss_pattern) if (l) lost++;
        packet_loss = loss_pattern.empty() ? 0.0 : (double)lost / loss_pattern.size();
        
        // Burst kayıp analizi
        size_t bursts = 0, burst_len = 0;
        for (bool l : loss_pattern) {
            if (l) {
                burst_len++;
                if (burst_len == 2) bursts++; // 2+ ardışık kayıp = burst
            } else {
                burst_len = 0;
            }
        }
        burst_loss = loss_pattern.empty() ? 0.0 : (double)bursts / loss_pattern.size();
    }
    
    void update_bandwidth(double mbps) {
        // EWMA ile bant genişliği güncelle
        bandwidth_mbps = (bandwidth_mbps > 0) 
            ? (bandwidth_mbps * 0.8 + mbps * 0.2) 
            : mbps;
    }
};

// Network profili
struct NetworkProfile {
    // MTU yönetimi
    struct MTUConfig {
        int base_mtu = 1200;     // Temel MTU boyutu
        int rtp_mtu = 1188;      // RTP header düşülmüş MTU
        int chunk_mtu = 1176;    // Chunk header düşülmüş MTU
        
        void update(int new_base) {
            base_mtu = new_base;
            rtp_mtu = base_mtu - RTP_HEADER_SIZE;
            chunk_mtu = rtp_mtu - sizeof(ChunkHeader);
        }

        friend std::ostream& operator<<(std::ostream& os, const MTUConfig& cfg) {
            return os << cfg.base_mtu;
        }
    } mtu;
    
    int jitter_buf_ms = 500;     // Jitter buffer latency
    int socket_buf_kb = 4096;    // Soket buffer boyutu (KB)
    int bitrate_kbps = 5000;     // Hedef bitrate
    
    // Profil skorlama (düşük = iyi)
    double score() const {
        UDPStats& stats = g_udp_stats; // global UDP istatistikleri
        double loss_penalty = stats.packet_loss * 100.0;
        double burst_penalty = stats.burst_loss * 200.0;
        double jitter_penalty = stats.jitter_ms * 0.5;
        double bw_penalty = (stats.bandwidth_mbps > 0) 
            ? std::max(0.0, (double)bitrate_kbps/1000.0 - stats.bandwidth_mbps) * 10.0
            : 0.0;
        return loss_penalty + burst_penalty + jitter_penalty + bw_penalty;
    }
    
    // MTU optimizasyonu
    void optimize_mtu() {
        UDPStats& stats = g_udp_stats;
        
        // Baz MTU seçimi (bitrate'e göre)
        int base_mtu = 1200;
        if (bitrate_kbps <= 6000)  base_mtu = 800;
        else if (bitrate_kbps <= 9000)  base_mtu = 960;
        else if (bitrate_kbps <= 12000) base_mtu = 1000;
        
        // Koşullara göre ayarla
        if (stats.packet_loss > 0.15 || stats.burst_loss > 0.05) {
            base_mtu = std::min(base_mtu, 800); // Yüksek kayıpta küçült
        }
        if (stats.jitter_ms > 50) {
            base_mtu = std::min(base_mtu, 900); // Yüksek jitter'da küçült
        }
        
        // Kademeli değişim (ani değişimlerden kaçın)
        int diff = base_mtu - mtu.base_mtu;
        if (std::abs(diff) >= 100) {
            mtu.update(mtu.base_mtu + ((diff > 0) ? 100 : -100));
        }
    }
    
    // Jitter buffer optimizasyonu
    void optimize_jitter_buffer() {
        UDPStats& stats = g_udp_stats;
        
        // RTT ve jitter bazlı hedef hesapla
        int target_latency = (int)(stats.avg_rtt_ms * 1.5 + stats.jitter_ms * 2.0);
        target_latency = std::max(100, std::min(2000, target_latency));
        
        // Burst kayıp varsa buffer'ı arttır
        if (stats.burst_loss > 0.02) {
            target_latency = (int)(target_latency * 1.2);
        }
        
        // Kademeli değişim
        int diff = target_latency - jitter_buf_ms;
        if (std::abs(diff) >= 50) {
            jitter_buf_ms += (diff > 0) ? 50 : -50;
        }
    }
    
    // Soket buffer optimizasyonu
    void optimize_socket_buffer() {
        UDPStats& stats = g_udp_stats;
        
        // Baz buffer boyutu (bitrate'e göre)
        int base_kb = 4096;
        if (bitrate_kbps > 10000) base_kb = 8192;
        if (bitrate_kbps > 20000) base_kb = 16384;
        
        // Jitter ve burst kayıp varsa arttır
        if (stats.jitter_ms > 30 || stats.burst_loss > 0.02) {
            base_kb = (int)(base_kb * 1.5);
        }
        
        // Kademeli değişim
        int diff = base_kb - socket_buf_kb;
        if (std::abs(diff) >= 1024) {
            socket_buf_kb += (diff > 0) ? 1024 : -1024;
        }
    }
    
    // Bitrate optimizasyonu
    void optimize_bitrate(int max_kbps) {
        UDPStats& stats = g_udp_stats;
        
        // Bant genişliği bazlı hedef
        int target = (stats.bandwidth_mbps > 0)
            ? (int)(stats.bandwidth_mbps * 1000.0 * 0.8) // %80 headroom
            : bitrate_kbps;
            
        // Kayıp ve jitter durumunda düşür
        if (stats.packet_loss > 0.1) {
            target = (int)(target * 0.8);
        }
        if (stats.jitter_ms > 50) {
            target = (int)(target * 0.9);
        }
        
        // Sınırlar
        target = std::max(2000, std::min(max_kbps, target));
        
        // Kademeli değişim
        int diff = target - bitrate_kbps;
        if (std::abs(diff) >= 500) {
            bitrate_kbps += (diff > 0) ? 500 : -500;
        }
    }
};

struct ChunkHeader {
    uint32_t magic = 0xABCD1234;
    uint32_t frame_id = 0;        // Video frame ID
    uint16_t chunk_index = 0;     // Chunk sıra no
    uint16_t total_chunks = 0;    // Frame'deki toplam chunk
    uint16_t k_data = 0;          // FEC data chunk sayısı
    uint16_t r_parity = 0;        // FEC parity chunk sayısı
    uint16_t payload_bytes = 0;   // Chunk payload boyutu
    uint32_t total_frame_bytes = 0;// Frame toplam boyutu
    uint32_t rtp_timestamp = 0;   // RTP ile senkronize timestamp
    uint16_t rtp_seq = 0;         // RTP sequence number
    uint8_t  flags = 0;           // bit0: parity, bit1: keyframe
    uint32_t checksum = 0;        // FNV-1a checksum
};
#pragma pack(pop)

struct ChunkSet {
    std::vector<std::vector<uint8_t>> data_chunks;
    std::vector<std::vector<uint8_t>> parity_chunks;
    int k = 0;
    int r = 0;
};

// RTP-FEC entegrasyon yöneticisi
class RTPFECManager {
private:
    uint16_t rtp_seq = 0;
    uint32_t rtp_timestamp = 0;
    std::atomic<uint32_t> frame_counter{0};
    
public:
    // RTP sequence number yönetimi
    uint16_t next_seq() {
        return ++rtp_seq;
    }
    
    // RTP timestamp yönetimi (90kHz clock)
    uint32_t update_timestamp() {
        uint64_t now = now_us();
        rtp_timestamp = (uint32_t)((now * RTP_CLOCK_RATE) / 1000000);
        return rtp_timestamp;
    }
    
    // Frame ID yönetimi
    uint32_t next_frame_id() {
        return frame_counter.fetch_add(1);
    }
    
    // Chunk oluşturma ve FEC kodlama
    ChunkSet create_chunks(const uint8_t* data, size_t size, bool is_keyframe) {
        uint32_t frame_id = next_frame_id();
        uint32_t ts = update_timestamp();
        
        ChunkSet chunks = build_chunks_with_fec(data, size, 
            g_net_profile.mtu.chunk_mtu, frame_id, is_keyframe);
            
        // RTP bilgilerini ekle
        for (auto& chunk : chunks.data_chunks) {
            ChunkHeader* hdr = reinterpret_cast<ChunkHeader*>(chunk.data());
            hdr->rtp_timestamp = ts;
            hdr->rtp_seq = next_seq();
        }
        
        for (auto& chunk : chunks.parity_chunks) {
            ChunkHeader* hdr = reinterpret_cast<ChunkHeader*>(chunk.data());
            hdr->rtp_timestamp = ts;
            hdr->rtp_seq = next_seq();
        }
        
        return chunks;
    }
};

// Entegre buffer yönetimi
class IntegratedBufferManager {
public:
    struct FrameBuffer {
        uint32_t frame_id = 0;
        uint32_t rtp_timestamp = 0;
        uint16_t k = 0, r = 0, m = 0;
        uint16_t payload_size = 0;
        uint32_t total_frame_bytes = 0;
        std::vector<std::vector<uint8_t>> data_blocks;
        std::vector<std::vector<uint8_t>> parity_blocks;
        std::vector<uint8_t> data_present;
        std::vector<uint8_t> parity_present;
        std::chrono::steady_clock::time_point first_seen;
        bool is_keyframe = false;

        void init(const ChunkHeader& hdr) {
            frame_id = hdr.frame_id;
            k = hdr.k_data;
            r = hdr.r_parity;
            m = hdr.total_chunks;
            payload_size = hdr.payload_bytes;
            total_frame_bytes = hdr.total_frame_bytes;
            is_keyframe = (hdr.flags & 0x02) != 0;
            
            data_blocks.resize(k);
            parity_blocks.resize(r);
            data_present.assign(k, 0);
            parity_present.assign(r, 0);
            first_seen = std::chrono::steady_clock::now();
        }
    };
    
    std::mutex mtx;
    std::map<uint32_t, FrameBuffer> frames;  // frame_id -> buffer
    std::chrono::milliseconds max_latency{1000};
    uint32_t last_decoded_frame_id = 0;
    
public:
    void set_max_latency(std::chrono::milliseconds ms) {
        max_latency = ms;
    }
    
    void process_chunk(const std::vector<uint8_t>& chunk_data) {
        if (chunk_data.size() < sizeof(ChunkHeader)) return;
        
        const ChunkHeader* hdr = reinterpret_cast<const ChunkHeader*>(chunk_data.data());
        if (hdr->magic != 0xABCD1234) return;
        
        std::lock_guard<std::mutex> lock(mtx);
        
        // Frame buffer'ı al veya oluştur
        auto& fb = frames[hdr->frame_id];
        if (fb.frame_id == 0) {
            fb.frame_id = hdr->frame_id;
            fb.rtp_timestamp = hdr->rtp_timestamp;
            fb.k = hdr->k_data;
            fb.r = hdr->r_parity;
            fb.m = hdr->total_chunks;
            fb.payload_size = hdr->payload_bytes;
            fb.total_frame_bytes = hdr->total_frame_bytes;
            fb.first_seen = std::chrono::steady_clock::now();
            fb.is_keyframe = (hdr->flags & 0x02) != 0;
            
            fb.data_blocks.resize(fb.k);
            fb.parity_blocks.resize(fb.r);
            fb.data_present.assign(fb.k, 0);
            fb.parity_present.assign(fb.r, 0);
        }
        
        // Chunk'ı doğru yere yerleştir
        const uint8_t* payload = chunk_data.data() + sizeof(ChunkHeader);
        size_t payload_size = chunk_data.size() - sizeof(ChunkHeader);
        
        if (hdr->flags & 0x01) {
            // Parity chunk
            uint16_t idx = hdr->chunk_index - fb.k;
            if (idx < fb.r) {
                fb.parity_blocks[idx].assign(payload, payload + payload_size);
                fb.parity_present[idx] = 1;
            }
        } else {
            // Data chunk
            if (hdr->chunk_index < fb.k) {
                fb.data_blocks[hdr->chunk_index].assign(payload, payload + payload_size);
                fb.data_present[hdr->chunk_index] = 1;
            }
        }
        
        cleanup_old_frames();
    }
    
    bool try_decode_frame(uint32_t frame_id, std::vector<uint8_t>& out_data) {
        std::lock_guard<std::mutex> lock(mtx);
        
        auto it = frames.find(frame_id);
        if (it == frames.end()) return false;
        
        auto& fb = it->second;
        if (try_reconstruct_frame(fb, out_data)) {
            frames.erase(it);
            last_decoded_frame_id = frame_id;
            return true;
        }
        return false;
    }
    
private:
    void cleanup_old_frames() {
        auto now = std::chrono::steady_clock::now();
        for (auto it = frames.begin(); it != frames.end();) {
            if (now - it->second.first_seen > max_latency) {
                it = frames.erase(it);
            } else {
                ++it;
            }
        }
    }
};

static bool try_reconstruct_frame(IntegratedBufferManager::FrameBuffer& fb, std::vector<uint8_t>& out_bytes) {
    const int k = fb.k, r = fb.r, m = fb.m, len = fb.payload_size;

    int have_data = 0; for (int i=0; i<k; ++i) have_data += fb.data_present[i]?1:0;
    if (have_data == k) {
        out_bytes.resize(fb.total_frame_bytes);
        size_t off = 0;
        for (int i=0; i<k; ++i) {
            size_t to_copy = std::min<size_t>(len, fb.total_frame_bytes - off);
            if (!to_copy) break;
            std::memcpy(out_bytes.data()+off, fb.data_blocks[i].data(), to_copy);
            off += to_copy;
        }
        return true;
    }

    int present_total = have_data; for (int i=0; i<r; ++i) present_total += fb.parity_present[i]?1:0;
    int missing_data = k - have_data;
    if (present_total < k || missing_data > r) return false;

    std::vector<uint8_t> A((size_t)m*(size_t)k); gf_gen_rs_matrix(A.data(), m, k);
    std::vector<uint8_t> frag_err; frag_err.reserve(k);
    for (int i=0; i<k; ++i) if (!fb.data_present[i]) frag_err.push_back((uint8_t)i);
    int nerrs = (int)frag_err.size(); if (nerrs==0) return false;

    std::vector<uint8_t> survive; survive.reserve(k);
    for (int i=0; i<k && (int)survive.size()<k; ++i) if (fb.data_present[i]) survive.push_back((uint8_t)i);
    for (int i=0; i<r && (int)survive.size()<k; ++i) if (fb.parity_present[i]) survive.push_back((uint8_t)(k+i));
    if ((int)survive.size() < k) return false;

    std::vector<uint8_t> B((size_t)k*(size_t)k);
    for (int row=0; row<k; ++row) {
        uint8_t idx = survive[row];
        for (int col=0; col<k; ++col) B[(size_t)k*row+col] = A[(size_t)k*idx+col];
    }
    std::vector<uint8_t> Binv((size_t)k*(size_t)k);
    if (gf_invert_matrix(B.data(), Binv.data(), k) < 0) return false;

    std::vector<uint8_t> D((size_t)k*(size_t)nerrs);
    for (int e=0; e<nerrs; ++e) {
        int err_idx = frag_err[e];
        for (int j=0; j<k; ++j) D[(size_t)k*e + j] = Binv[(size_t)k*err_idx + j];
    }

    std::vector<uint8_t*> srcs((size_t)k,nullptr);
    for (int row=0; row<k; ++row) {
        uint8_t idx = survive[row];
        srcs[row] = (idx < k) ? fb.data_blocks[idx].data() : fb.parity_blocks[idx-k].data();
    }

    std::vector<std::vector<uint8_t>> out((size_t)nerrs, std::vector<uint8_t>(len));
    std::vector<uint8_t*> outp((size_t)nerrs);
    for (int i=0; i<nerrs; ++i) outp[i] = out[i].data();

    std::vector<uint8_t> g_tbls((size_t)k*(size_t)nerrs*32);
    ec_init_tables(k, nerrs, D.data(), g_tbls.data());
    ec_encode_data(len, k, nerrs, g_tbls.data(), srcs.data(), outp.data());

    for (int i=0; i<nerrs; ++i) {
        int idx = frag_err[i];
        fb.data_blocks[(size_t)idx] = std::move(out[(size_t)i]);
        fb.data_present[(size_t)idx] = 1;
    }
    int have = 0; for (int i=0; i<k; ++i) have += fb.data_present[i]?1:0;
    if (have != k) return false;

    out_bytes.resize(fb.total_frame_bytes);
    size_t off = 0;
    for (int i=0; i<k; ++i) {
        size_t to_copy = std::min<size_t>(len, fb.total_frame_bytes - off);
        if (!to_copy) break;
        std::memcpy(out_bytes.data()+off, fb.data_blocks[i].data(), to_copy);
        off += to_copy;
    }
    return true;
}

// RTP zaman yönetimi
class RTPTimestampManager {
private:
    uint32_t last_rtp_ts = 0;
    uint64_t last_wall_ts = 0;
    std::mutex mtx;
    
public:
    static inline uint64_t now_us() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }
    
    uint32_t get_rtp_timestamp() {
        std::lock_guard<std::mutex> lock(mtx);
        uint64_t now = now_us();
        
        // İlk çağrı
        if (last_wall_ts == 0) {
            last_wall_ts = now;
            last_rtp_ts = (uint32_t)((now * RTP_CLOCK_RATE) / 1000000);
            return last_rtp_ts;
        }
        
        // Geçen süreyi RTP clock'a çevir
        uint64_t delta_us = now - last_wall_ts;
        uint32_t delta_rtp = (uint32_t)((delta_us * RTP_CLOCK_RATE) / 1000000);
        
        // Wrap-around kontrolü
        last_rtp_ts = (last_rtp_ts + delta_rtp) & 0xFFFFFFFF;
        last_wall_ts = now;
        
        return last_rtp_ts;
    }
};

static RTPTimestampManager g_rtp_time_mgr;

static ChunkSet build_chunks_with_fec(const uint8_t* data, size_t size, size_t mtu_bytes, 
                                    uint32_t frame_id, bool is_keyframe) {
    ChunkSet out{};
    const size_t H = sizeof(ChunkHeader);
    if (mtu_bytes <= H) return out;

    const size_t payload = mtu_bytes - H;
    int k = (int)((size + payload - 1) / payload);
    if (k <= 0) k = 1;

    // Adaptif FEC oranı: taban %12, loss ile art, cap %35
    double avg_loss = 0.0;
    if (!g_last_stats.empty()) {
        for (auto& s : g_last_stats) avg_loss += s.packet_loss;
        avg_loss /= g_last_stats.size();
    }
    double base = 0.12, cap = 0.35;
    double fec_ratio = std::min(cap, base + std::max(0.01, avg_loss) * 1.2);
    int r = std::clamp((int)std::ceil(k * fec_ratio), 1, std::max(3, k/3));
    if (is_keyframe) r = std::min(r + 1, std::max(4, k/2)); // keyframe'e bonus

    out.k = k; out.r = r;

    auto fnv1a = [](const uint8_t* p, size_t n) {
        uint32_t h = 2166136261u;
        for (size_t i=0; i<n; ++i) { h ^= p[i]; h *= 16777619u; }
        return h;
    };

    out.data_chunks.resize(k);
    size_t off = 0;
    for (int i=0; i<k; ++i) {
        out.data_chunks[i].assign(mtu_bytes, 0);
        ChunkHeader h{};
        h.frame_id = frame_id; h.chunk_index = (uint16_t)i;
        h.total_chunks = (uint16_t)(k+r); h.k_data = (uint16_t)k; h.r_parity = (uint16_t)r;
        h.payload_bytes = (uint16_t)payload; h.total_frame_bytes = (uint32_t)size;
        h.timestamp_us = now_us(); h.flags = is_keyframe ? 0x02 : 0x00;
        std::memcpy(out.data_chunks[i].data(), &h, sizeof(ChunkHeader));

        size_t rem = (off < size) ? (size - off) : 0;
        size_t cp = std::min(payload, rem);
        if (cp) std::memcpy(out.data_chunks[i].data()+H, data+off, cp);
        h.checksum = fnv1a(out.data_chunks[i].data()+H, payload);
        std::memcpy(out.data_chunks[i].data(), &h, sizeof(ChunkHeader));
        off += cp;
    }

    if (k >= 2) {
        std::vector<uint8_t*> dptr((size_t)k);
        for (int i=0; i<k; ++i) dptr[i] = out.data_chunks[i].data() + H;

        std::vector<std::vector<uint8_t>> pbuf((size_t)r, std::vector<uint8_t>(payload));
        std::vector<uint8_t*> pptr((size_t)r);
        for (int i=0; i<r; ++i) pptr[i] = pbuf[i].data();

        std::vector<uint8_t> M((size_t)(k+r)*(size_t)k);
        std::vector<uint8_t> tbl((size_t)k*(size_t)r*32);
        gf_gen_rs_matrix(M.data(), k+r, k);
        ec_init_tables(k, r, M.data()+k*k, tbl.data());
        ec_encode_data((int)payload, k, r, tbl.data(), dptr.data(), pptr.data());

        out.parity_chunks.resize(r);
        for (int i=0; i<r; ++i) {
            out.parity_chunks[i].assign(mtu_bytes, 0);
            ChunkHeader h{};
            h.frame_id = frame_id; h.chunk_index = (uint16_t)(k+i);
            h.total_chunks = (uint16_t)(k+r); h.k_data = (uint16_t)k; h.r_parity = (uint16_t)r;
            h.payload_bytes = (uint16_t)payload; h.total_frame_bytes = (uint32_t)size;
            h.timestamp_us = now_us(); h.flags = (uint8_t)(0x01 | (is_keyframe?0x02:0x00));
            std::memcpy(out.parity_chunks[i].data(), &h, sizeof(ChunkHeader));
            std::memcpy(out.parity_chunks[i].data()+H, pbuf[i].data(), payload);
            h.checksum = fnv1a(out.parity_chunks[i].data()+H, payload);
            std::memcpy(out.parity_chunks[i].data(), &h, sizeof(ChunkHeader));
        }
    }
    return out;
}

#include <tuple>

static std::atomic<bool> g_stop(false);
static GMainLoop* g_loop = nullptr;
static NetworkProfile g_net_profile;
static std::vector<UDPStats> g_last_stats;
static UDPStats g_udp_stats;
static GstElement* g_rtp_pay = nullptr;
static std::shared_ptr<SharedUDPSocket> g_shared_socket;

// ---------- Headless/ENV helper ----------
static void ensure_runtime_env() {
  // Wayland kullanan bazı sink'ler XDG_RUNTIME_DIR ister.
  if (!getenv("XDG_RUNTIME_DIR")) {
    char buf[64];
    snprintf(buf, sizeof(buf), "/run/user/%d", getuid());
    setenv("XDG_RUNTIME_DIR", buf, 0); // yoksa set et (0: var ise dokunma)
  }
}
static bool headless_env() {
  const char* d = getenv("DISPLAY");
  const char* w = getenv("WAYLAND_DISPLAY");
  const char* xdg = getenv("XDG_RUNTIME_DIR");
  // GUI yoksa ya da XDG yoksa headless kabul edelim
  return ((!d && !w) || !(xdg && *xdg));
}

static void sig_handler(int){
  g_stop = true;
  if (g_loop) g_main_loop_quit(g_loop);
}

struct Args {
  std::string peer_ip;

  // === tek port tasarımı ===
  int media_port; // hem send hem recv (RTP)
  int ctrl_port;  // hem send hem recv (PING/PONG)

  bool use_ts = false;
  int  mtu = 1000;          // daha güvenli başlangıç
  int  bitrate_kbps = 12000;// dalgalanan LAN için daha stabil
  int  keyint = 30;         // toparlama hızlı
  int  latency_ms = 800;    // başlangıç jitter

  std::string device = "/dev/video0";
  int width = 1280, height = 720, fps = 30;
  int prefer_mjpg = 1;
};

struct CamProfile {
  std::string device;
  int width = 0, height = 0, fps = 0;
  bool mjpg = false;
  long long score() const { return 1LL * width * height * fps; }
};

static constexpr int MAX_W = 7680;
static constexpr int MAX_H = 4320;
static constexpr int MAX_FPS = 240;

static const int PREFERRED_MODES[][3] = {
  {3840,2160,60}, {3840,2160,30},
  {2560,1440,60}, {2560,1440,30},
  {1920,1080,60}, {1920,1080,30},
  {1600, 900,60}, {1600, 900,30},
  {1280, 720,60}, {1280, 720,30},
  {960, 540,30},
  {848, 480,30},
  {640, 480,30},
};
static constexpr int PREFERRED_COUNT = sizeof(PREFERRED_MODES)/sizeof(PREFERRED_MODES[0]);

// ---- helpers to read caps ranges ----
static bool get_int_min_max(const GValue* v, int& out_min, int& out_max) {
  if (!v) return false;
  if (G_VALUE_HOLDS_INT(v)) { out_min = out_max = g_value_get_int(v); return true; }
  if (GST_VALUE_HOLDS_INT_RANGE(v)) {
    out_min = gst_value_get_int_range_min(v);
    out_max = gst_value_get_int_range_max(v);
    if (out_max > MAX_W * 4) out_max = MAX_W;
    if (out_min < 1) out_min = 1;
    return true;
  }
  if (GST_VALUE_HOLDS_LIST(v)) {
    int mn = INT_MAX, mx = 0, ok = 0;
    for (guint i=0;i<gst_value_list_get_size(v);++i) {
      const GValue* it = gst_value_list_get_value(v, i);
      int a=0,b=0;
      if (get_int_min_max(it, a, b)) { mn = std::min(mn, a); mx = std::max(mx, b); ok++; }
    }
    if (ok) { out_min = mn; out_max = mx; return true; }
  }
  return false;
}

static bool get_fps_min_max(const GValue* v, int& out_min, int& out_max) {
  auto frac_to_int = [](const GValue* fv)->int{
    int n = gst_value_get_fraction_numerator(fv);
    int d = gst_value_get_fraction_denominator(fv);
    if (d<=0) return 0;
    return n/d;
  };
  if (!v) { out_min = 1; out_max = 30; return true; }

  if (GST_VALUE_HOLDS_FRACTION(v)) {
    int f = frac_to_int(v);
    if (f<=0) return false;
    out_min = out_max = std::min(f, MAX_FPS);
    return true;
  }
  if (GST_VALUE_HOLDS_FRACTION_RANGE(v)) {
    const GValue* minv = gst_value_get_fraction_range_min(v);
    const GValue* maxv = gst_value_get_fraction_range_max(v);
    int fmin = frac_to_int(minv);
    int fmax = frac_to_int(maxv);
    if (fmax<=0) return false;
    out_min = std::max(1, fmin);
    out_max = std::min(MAX_FPS, fmax);
    return true;
  }
  if (GST_VALUE_HOLDS_LIST(v)) {
    int mn = INT_MAX, mx = 0, ok=0;
    for (guint i=0;i<gst_value_list_get_size(v);++i) {
      const GValue* it = gst_value_list_get_value(v, i);
      int a=0,b=0;
      if (get_fps_min_max(it, a, b)) { mn = std::min(mn, a); mx = std::max(mx, b); ok++; }
    }
    if (ok) { out_min = mn; out_max = mx; return true; }
  }
  out_min = 1; out_max = 30;
  return true;
}

// --- caps enumeration & validation ---
struct CapsWindow { int wmin,wmax,hmin,hmax,fmin,fmax; bool mjpg; };

static std::vector<CapsWindow> enumerate_caps(const std::string& devpath) {
  std::vector<CapsWindow> out;
  GstElement* src = gst_element_factory_make("v4l2src", NULL);
  if (!src) return out;
  g_object_set(G_OBJECT(src), "device", devpath.c_str(), NULL);

  GstPad* pad = gst_element_get_static_pad(src, "src");
  if (!pad) { gst_object_unref(src); return out; }

  GstCaps* caps = gst_pad_query_caps(pad, NULL);
  if (!caps) { gst_object_unref(pad); gst_object_unref(src); return out; }

  const guint n = gst_caps_get_size(caps);
  for (guint i=0; i<n; ++i) {
    const GstStructure* s = gst_caps_get_structure(caps, i);
    const char* name = gst_structure_get_name(s);
    bool is_mjpg = g_str_has_prefix(name, "image/jpeg");

    int wmin=0,wmax=0,hmin=0,hmax=0, fmin=1,fmax=30;
    if (!get_int_min_max(gst_structure_get_value(s, "width"),  wmin, wmax)) continue;
    if (!get_int_min_max(gst_structure_get_value(s, "height"), hmin, hmax)) continue;
    get_fps_min_max(gst_structure_get_value(s, "framerate"), fmin, fmax);

    wmax = std::min(wmax, MAX_W);
    hmax = std::min(hmax, MAX_H);
    fmax = std::min(fmax, MAX_FPS);

    out.push_back({wmin,wmax,hmin,hmax,fmin,fmax,is_mjpg});
  }

  gst_caps_unref(caps);
  gst_object_unref(pad);
  gst_object_unref(src);
  return out;
}

static bool validate_mode(const std::string& devpath, bool mjpg, int W, int H, int F) {
  GstElement* pipe = gst_pipeline_new("probe");
  if (!pipe) return false;

  GstElement* src = gst_element_factory_make("v4l2src", "src");
  GstElement* capsf = gst_element_factory_make("capsfilter", "caps");
  GstElement* dec  = mjpg ? gst_element_factory_make("jpegdec", "jpegdec") : NULL;
  GstElement* conv = gst_element_factory_make("videoconvert", "conv");
  GstElement* sink = gst_element_factory_make("fakesink", "sink");
  if (!src || !capsf || !conv || !sink || (mjpg && !dec)) { if (pipe) gst_object_unref(pipe); return false; }

  g_object_set(G_OBJECT(src), "device", devpath.c_str(), NULL);

  GstCaps* caps = mjpg
    ? gst_caps_new_simple("image/jpeg", "width", G_TYPE_INT, W, "height", G_TYPE_INT, H,
                          "framerate", GST_TYPE_FRACTION, F, 1, NULL)
    : gst_caps_new_simple("video/x-raw", "width", G_TYPE_INT, W, "height", G_TYPE_INT, H,
                          "framerate", GST_TYPE_FRACTION, F, 1, NULL);

  g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
  gst_caps_unref(caps);
  g_object_set(G_OBJECT(sink), "sync", FALSE, NULL);

  if (mjpg) gst_bin_add_many(GST_BIN(pipe), src, capsf, dec, conv, sink, NULL);
  else      gst_bin_add_many(GST_BIN(pipe), src, capsf, conv, sink, NULL);

  gboolean linked = FALSE;
  if (mjpg) linked = gst_element_link_many(src, capsf, dec, conv, sink, NULL);
  else      linked = gst_element_link_many(src, capsf, conv, sink, NULL);
  if (!linked) { gst_object_unref(pipe); return false; }

  GstStateChangeReturn r = gst_element_set_state(pipe, GST_STATE_PLAYING);
  if (r == GST_STATE_CHANGE_ASYNC) r = gst_element_get_state(pipe, NULL, NULL, GST_SECOND);

  bool ok = (r == GST_STATE_CHANGE_SUCCESS || r == GST_STATE_CHANGE_NO_PREROLL);
  gst_element_set_state(pipe, GST_STATE_NULL);
  gst_object_unref(pipe);
  return ok;
}

static std::optional<CamProfile> probe_device_best(const std::string& devpath) {
  auto windows = enumerate_caps(devpath);
  if (windows.empty()) return std::nullopt;

  auto try_space = [&](bool mjpg)->std::optional<CamProfile>{
    for (int k=0; k<PREFERRED_COUNT; ++k) {
      int W = PREFERRED_MODES[k][0];
      int H = PREFERRED_MODES[k][1];
      int F = PREFERRED_MODES[k][2];
      for (const auto& cw : windows) {
        if (cw.mjpg != mjpg) continue;
        if (W>=cw.wmin && W<=cw.wmax && H>=cw.hmin && H<=cw.hmax && F>=cw.fmin && F<=cw.fmax) {
          if (validate_mode(devpath, mjpg, W,H,F)) {
            return CamProfile{devpath, W,H,F, mjpg};
          }
        }
      }
    }
    return std::nullopt;
  };

  if (auto ok = try_space(true))  return ok;   // MJPG
  if (auto ok = try_space(false)) return ok;   // RAW

  if (validate_mode(devpath, true, 1280,720,30))  return CamProfile{devpath,1280,720,30,true};
  if (validate_mode(devpath, false,1280,720,30))  return CamProfile{devpath,1280,720,30,false};
  return std::nullopt;
}

static bool auto_select_best_camera(Args& a) {
  CamProfile best;
  bool found = false;
  for (int n=0; n<=9; ++n) {
    std::string dev = "/dev/video" + std::to_string(n);
    auto prof = probe_device_best(dev);
    if (!prof.has_value()) continue;
    if (!found || prof->score() > best.score() || (prof->score()==best.score() && prof->mjpg && !best.mjpg)) {
      best = *prof; found = true;
    }
  }
  if (!found) return false;

  a.device      = best.device;
  a.width       = best.width;
  a.height      = best.height;
  a.fps         = best.fps;
  a.prefer_mjpg = best.mjpg ? 1 : 0;

  std::cout << "[auto] device=" << a.device
            << " mode=" << (a.prefer_mjpg ? "MJPG" : "RAW")
            << " " << a.width << "x" << a.height
            << "@" << a.fps << " selected\n";
  return true;
}

// =================== ADAPTİF DURUMLAR & ÖLÇÜMLER ===================
static std::atomic<int> g_rtt_ewma_ms{0};   // kontrol kanalından EWMA RTT(app)
static std::atomic<int> g_local_rx_kbps{0}; // bu uçta ölçülen alınan kbps (receiver identity)
static std::atomic<int> g_peer_rx_kbps{0};  // karşı uçtan gelen RX raporu (STAT)

// Soket monitoring thread
static void socket_monitor_thread() {
    while (!g_stop) {
        if (g_shared_socket) {
            // İstatistikleri güncelle
            g_shared_socket->calculate_stats();
            
            // İstatistikleri logla
            auto stats = g_shared_socket->get_stats();
            std::cout << "[socket] TX: " << stats.tx_mbps << " Mbps (" 
                     << stats.tx_pps << " pps), RX: " << stats.rx_mbps 
                     << " Mbps (" << stats.rx_pps << " pps), "
                     << "Avg packet: " << stats.avg_packet_size << " bytes";
            
            // Congestion kontrolü
            if (g_shared_socket->is_congested()) {
                std::cout << " [CONGESTED]";
                
                // MTU ve buffer ayarlarını düşür
                if (g_net_profile.mtu.base_mtu > 800) {
                    g_net_profile.mtu.update(g_net_profile.mtu.base_mtu - 100);
                }
            }
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Genişletilmiş UDP profil metrikleri
struct UDPStats {
    double packet_loss = 0.0;    // paket kayıp oranı
    double avg_rtt_ms = 0.0;     // ortalama RTT
    double jitter_ms = 0.0;      // jitter (RTT varyansı)
    double burst_loss = 0.0;     // ardışık kayıp oranı
    double bandwidth_mbps = 0.0; // tahmini bant genişliği
    
    std::vector<double> rtt_samples;  // RTT örnekleri (jitter için)
    std::vector<bool> loss_pattern;   // kayıp paterni (burst için)
    static constexpr size_t MAX_SAMPLES = 100;
    
    void update_rtt(double rtt_ms) {
        // EWMA ile RTT güncelle
        avg_rtt_ms = (avg_rtt_ms > 0) ? (avg_rtt_ms * 0.8 + rtt_ms * 0.2) : rtt_ms;
        
        // Jitter hesapla
        rtt_samples.push_back(rtt_ms);
        if (rtt_samples.size() > MAX_SAMPLES) rtt_samples.erase(rtt_samples.begin());
        if (rtt_samples.size() >= 2) {
            double sum = 0, mean = avg_rtt_ms;
            for (double s : rtt_samples) sum += (s - mean) * (s - mean);
            jitter_ms = std::sqrt(sum / rtt_samples.size());
        }
    }
    
    void update_loss(bool is_lost) {
        // Kayıp paterni güncelle
        loss_pattern.push_back(is_lost);
        if (loss_pattern.size() > MAX_SAMPLES) loss_pattern.erase(loss_pattern.begin());
        
        // Genel kayıp oranı
        size_t lost = 0;
        for (bool l : loss_pattern) if (l) lost++;
        packet_loss = loss_pattern.empty() ? 0.0 : (double)lost / loss_pattern.size();
        
        // Burst kayıp analizi
        size_t bursts = 0, burst_len = 0;
        for (bool l : loss_pattern) {
            if (l) {
                burst_len++;
                if (burst_len == 2) bursts++; // 2+ ardışık kayıp = burst
            } else {
                burst_len = 0;
            }
        }
        burst_loss = loss_pattern.empty() ? 0.0 : (double)bursts / loss_pattern.size();
    }
    
    void update_bandwidth(double mbps) {
        // EWMA ile bant genişliği güncelle
        bandwidth_mbps = (bandwidth_mbps > 0) 
            ? (bandwidth_mbps * 0.8 + mbps * 0.2) 
            : mbps;
    }
};

struct RateMeter { std::atomic<long long> bytes{0}; };

static void on_handoff_meter(GstElement* /*identity*/, GstBuffer* buffer, GstPad* /*pad*/, gpointer user_data) {
  auto* m = static_cast<RateMeter*>(user_data);
  if (!m || !buffer) return;
  gsize sz = gst_buffer_get_size(buffer);
  m->bytes.fetch_add((long long)sz, std::memory_order_relaxed);
}

static void rate_meter_loop(RateMeter* meter, std::atomic<int>* out_kbps) {
  int ewma = 0;
  while (!g_stop) {
    long long bytes = meter->bytes.exchange(0);
    int kbps = (int)((bytes * 8) / 1000);
    ewma = (ewma > 0) ? (ewma * 7 + kbps * 3) / 10 : kbps;
    out_kbps->store(ewma);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// --------- Gelişmiş UDP Soket Yönetimi ----------
class SharedUDPSocket {
private:
    GSocket* socket;
    std::atomic<uint64_t> tx_bytes{0};
    std::atomic<uint64_t> rx_bytes{0};
    std::atomic<uint32_t> tx_packets{0};
    std::atomic<uint32_t> rx_packets{0};
    std::chrono::steady_clock::time_point last_stats_reset;
    
    // Performans metrikleri
    struct SocketStats {
        double tx_mbps = 0.0;
        double rx_mbps = 0.0;
        uint32_t tx_pps = 0;
        uint32_t rx_pps = 0;
        double avg_packet_size = 0.0;
    };
    SocketStats current_stats;
    std::mutex stats_mtx;
    
public:
    SharedUDPSocket(int port) {
        GError* err = nullptr;
        socket = g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM, G_SOCKET_PROTOCOL_UDP, &err);
        if (!socket) {
            std::string error_msg = err ? err->message : "Unknown error";
            if (err) g_error_free(err);
            throw std::runtime_error("Socket creation failed: " + error_msg);
        }
        
        // 1. Non-blocking mod
        g_socket_set_blocking(socket, FALSE);
        
        // 2. Optimize edilmiş buffer boyutları
        const int send_buf = 8 * 1024 * 1024; // 8MB
        const int recv_buf = 8 * 1024 * 1024; // 8MB
        g_socket_set_option(socket, SOL_SOCKET, SO_SNDBUF, send_buf, nullptr);
        g_socket_set_option(socket, SOL_SOCKET, SO_RCVBUF, recv_buf, nullptr);
        
        // 3. Socket seçenekleri
        g_socket_set_option(socket, SOL_SOCKET, SO_REUSEADDR, 1, nullptr);
        #ifdef SO_REUSEPORT
        g_socket_set_option(socket, SOL_SOCKET, SO_REUSEPORT, 1, nullptr);
        #endif
        
        // 4. QoS ayarları
        const int tos = 0xb8; // DSCP EF (Expedited Forwarding)
        g_socket_set_option(socket, IPPROTO_IP, IP_TOS, tos, nullptr);
        
        // 5. Receive timestamping
        #ifdef SO_TIMESTAMP
        g_socket_set_option(socket, SOL_SOCKET, SO_TIMESTAMP, 1, nullptr);
        #endif
        
        // 6. UDP segmentation offload
        #ifdef UDP_SEGMENT
        g_socket_set_option(socket, IPPROTO_UDP, UDP_SEGMENT, 1, nullptr);
        #endif
        
        // 7. Bind
        GInetAddress* any = g_inet_address_new_any(G_SOCKET_FAMILY_IPV4);
        GSocketAddress* sa = g_inet_socket_address_new(any, port);
        if (!g_socket_bind(socket, sa, TRUE, &err)) {
            g_object_unref(any);
            g_object_unref(sa);
            std::string error_msg = err ? err->message : "Unknown error";
            if (err) g_error_free(err);
            throw std::runtime_error("Socket bind failed: " + error_msg);
        }
        g_object_unref(any);
        g_object_unref(sa);
        
        last_stats_reset = std::chrono::steady_clock::now();
    }
    
    // İstatistik toplama
    void update_stats(size_t bytes, bool is_tx) {
        if (is_tx) {
            tx_bytes.fetch_add(bytes);
            tx_packets.fetch_add(1);
        } else {
            rx_bytes.fetch_add(bytes);
            rx_packets.fetch_add(1);
        }
    }
    
    // Periyodik istatistik hesaplama
    void calculate_stats() {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_reset);
        if (duration.count() < 1000) return; // 1 saniyede bir hesapla
        
        std::lock_guard<std::mutex> lock(stats_mtx);
        
        double seconds = duration.count() / 1000.0;
        
        uint64_t tx_b = tx_bytes.exchange(0);
        uint64_t rx_b = rx_bytes.exchange(0);
        uint32_t tx_p = tx_packets.exchange(0);
        uint32_t rx_p = rx_packets.exchange(0);
        
        current_stats.tx_mbps = (tx_b * 8.0) / (seconds * 1000000.0);
        current_stats.rx_mbps = (rx_b * 8.0) / (seconds * 1000000.0);
        current_stats.tx_pps = static_cast<uint32_t>(tx_p / seconds);
        current_stats.rx_pps = static_cast<uint32_t>(rx_p / seconds);
        current_stats.avg_packet_size = (tx_p + rx_p > 0) ? 
            static_cast<double>(tx_b + rx_b) / (tx_p + rx_p) : 0.0;
            
        last_stats_reset = now;
    }
    
    // Yük dengeleme kontrolü
    bool is_congested() const {
        std::lock_guard<std::mutex> lock(stats_mtx);
        return (current_stats.tx_pps + current_stats.rx_pps) > 10000 || // 10K pps
               (current_stats.tx_mbps + current_stats.rx_mbps) > 800.0; // 800 Mbps
    }
    
    // İstatistik raporu
    SocketStats get_stats() const {
        std::lock_guard<std::mutex> lock(stats_mtx);
        return current_stats;
    }
    
    // Socket erişimi
    GSocket* get() const { return socket; }
    
    ~SharedUDPSocket() {
        if (socket) g_object_unref(socket);
    }
};

// Global soket instance
static std::shared_ptr<SharedUDPSocket> g_shared_socket;

// Eski API için wrapper (geriye uyumluluk)
static GSocket* create_shared_udp_socket(int port) {
    try {
        g_shared_socket = std::make_shared<SharedUDPSocket>(port);
        return g_shared_socket->get();
    } catch (const std::exception& e) {
        std::cerr << "[udp] Socket creation failed: " << e.what() << "\n";
        return nullptr;
    }
}

// =================== KONTROL KANALI: PING/PONG + STAT ===================
class ControlChannel {
 public:
  ControlChannel(const std::string& peer_ip, int port)
  : peer_ip_(peer_ip), port_(port) {}

  bool start() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) { perror("socket ctrl"); return false; }

    int yes=1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
    setsockopt(fd_, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif
    sockaddr_in addr{}; addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);
    if (bind(fd_, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind ctrl"); return false; }

    memset(&peer_addr_, 0, sizeof(peer_addr_));
    peer_addr_.sin_family = AF_INET;
    peer_addr_.sin_port = htons(port_);
    inet_pton(AF_INET, peer_ip_.c_str(), &peer_addr_.sin_addr);

    recv_thr_ = std::thread([this]{ this->recv_loop(); });
    send_thr_ = std::thread([this]{ this->send_loop(); });
    return true;
  }
  void stop() {
    g_stop = true;
    if (send_thr_.joinable()) send_thr_.join();
    if (recv_thr_.joinable()) recv_thr_.join();
    if (fd_>=0) close(fd_);
  }

 private:
  void send_loop() {
    using namespace std::chrono;
    int tick = 0;
    while (!g_stop) {
      long long now = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
      char buf[64];
      int n = snprintf(buf, sizeof(buf), "PING %lld", now);
      sendto(fd_, buf, n, 0, (sockaddr*)&peer_addr_, sizeof(peer_addr_));

      if ((++tick & 1) == 0) { // ~1s'de bir STAT
        int rx = g_local_rx_kbps.load();
        int m = snprintf(buf, sizeof(buf), "STAT %d", rx);
        sendto(fd_, buf, m, 0, (sockaddr*)&peer_addr_, sizeof(peer_addr_));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  void recv_loop() {
    char buf[256];
    while (!g_stop) {
      sockaddr_in src{}; socklen_t sl = sizeof(src);
      int n = recvfrom(fd_, buf, sizeof(buf)-1, 0, (sockaddr*)&src, &sl);
      if (n <= 0) continue;
      buf[n] = 0;
      if (!strncmp(buf, "PING ", 5)) {
        buf[1] = 'O'; buf[2] = 'N'; buf[3] = 'G';
        sendto(fd_, buf, n, 0, (sockaddr*)&src, sizeof(src));
      } else if (!strncmp(buf, "PONG ", 5)) {
        long long t0 = atoll(buf+5);
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now().time_since_epoch()).count();
        int rtt = (int)(now - t0);
        std::cout << "[ctrl] RTT(app) ~ " << rtt << " ms\n";
        int prev = g_rtt_ewma_ms.load();
        int ewma = (prev > 0) ? (prev * 7 + rtt * 3) / 10 : rtt;
        g_rtt_ewma_ms.store(ewma);
      } else if (!strncmp(buf, "STAT ", 5)) {
        int rx = atoi(buf + 5); // kbps
        g_peer_rx_kbps.store(rx);
      }
    }
  }

  std::string peer_ip_;
  int port_;
  int fd_ = -1;
  sockaddr_in peer_addr_{};
  std::thread send_thr_, recv_thr_;
};

// =================== BUS & STDIN ===================
static gboolean bus_cb(GstBus* /*bus*/, GstMessage* msg, gpointer user_data) {
  const char* tag = static_cast<const char*>(user_data);
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError* err=nullptr; gchar* dbg=nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      std::cerr << "[" << tag << "] ERROR: " << (err?err->message:"") << (dbg?std::string(" | ")+dbg:"") << std::endl;
      if (err) g_error_free(err); if (dbg) g_free(dbg);
      g_stop = true; if (g_loop) g_main_loop_quit(g_loop);
      break;
    }
    case GST_MESSAGE_EOS:
      std::cerr << "[" << tag << "] EOS\n";
      g_stop = true; if (g_loop) g_main_loop_quit(g_loop);
      break;
    default: break;
  }
  return TRUE;
}

static gboolean stdin_cb(GIOChannel* ch, GIOCondition cond, gpointer) {
  if (cond & (G_IO_HUP|G_IO_ERR|G_IO_NVAL)) { return TRUE; }
  gchar buf[16]; gsize n=0; GError* err=nullptr;
  GIOStatus s = g_io_channel_read_chars(ch, buf, sizeof(buf), &n, &err);
  if (s == G_IO_STATUS_NORMAL && n>0) {
    for (gsize i=0;i<n;i++) {
      unsigned char c = (unsigned char)buf[i];
      if (c==27 || c=='q' || c=='Q') {
        std::cout << "[key] quit\n";
        g_stop = true; if (g_loop) g_main_loop_quit(g_loop);
        break;
      }
    }
  }
  if (err) g_error_free(err);
  return TRUE;
}

// =================== ADAPTİF JITTER (RTT'ye göre) ===================
static void adaptive_jbuf_loop(GstElement* jbuf) {
  g_object_ref(jbuf);
  int last_set = -1;
  while (!g_stop) {
    int rtt = g_rtt_ewma_ms.load();
    if (rtt > 0) {
      int target = rtt * 3 / 2 + 50; // 1.5*RTT + 50ms
      if (target < 150) target = 150;
      if (target > 4000) target = 4000;
      if (last_set < 0 || std::abs(target - last_set) >= 100) {
        g_object_set(G_OBJECT(jbuf), "latency", target, NULL);
        last_set = target;
        std::cout << "[jbuf] adapt latency=" << target
                  << " ms (rtt_ewma=" << rtt << ")\n";
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  g_object_unref(jbuf);
}

// thread-safe property set (main context üzerinden)
static void set_int_async(GObject* obj, const char* prop, int val) {
  GMainContext* ctx = g_main_context_default();
  g_main_context_invoke(ctx, [](gpointer data)->gboolean {
    auto tup = static_cast<std::tuple<GObject*, const char*, int>*>(data);
    g_object_set(std::get<0>(*tup), std::get<1>(*tup), std::get<2>(*tup), NULL);
    delete tup;
    return G_SOURCE_REMOVE;
  }, new std::tuple<GObject*, const char*, int>(obj, prop, val));
}

static void set_bool_if_exists(GObject* obj, const char* prop, gboolean val) {
    GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(obj), prop);
    if (pspec) {
        g_object_set(obj, prop, val, NULL);
    }
}

static std::string choose_h264_encoder() {
    // GPU öncelik sırası: NVIDIA > VAAPI > QSV > x264
    if (has_nvidia_gpu()) {
        GstElement* enc = gst_element_factory_make("nvh264enc", nullptr);
        if (enc) {
            gst_object_unref(enc);
            return "nvh264enc";
        }
    }
    
    if (has_vaapi_gpu()) {
        GstElement* enc = gst_element_factory_make("vaapih264enc", nullptr);
        if (enc) {
            gst_object_unref(enc);
            return "vaapih264enc";
        }
    }
    
    if (has_qsv_gpu()) {
        GstElement* enc = gst_element_factory_make("qsvh264enc", nullptr);
        if (enc) {
            gst_object_unref(enc);
            return "qsvh264enc";
        }
    }
    
    return "x264enc"; // CPU fallback
}

static std::string g_last_encoder_name;

static void set_encoder_bitrate(GstElement* enc, int kbps) {
  const std::string& n = g_last_encoder_name;
  if (n == "qsvh264enc" || n == "vah264enc") {
    set_int_async(G_OBJECT(enc), "bitrate", kbps * 1000); // bps
  } else {
    set_int_async(G_OBJECT(enc), "bitrate", kbps);        // kbps
  }
}

// Gelişmiş MTU ve buffer yönetimi
struct NetworkProfile {
    // MTU yönetimi
    struct MTUConfig {
        int base_mtu = 1200;     // Temel MTU boyutu
        int rtp_mtu = 1188;      // RTP header düşülmüş MTU
        int chunk_mtu = 1176;    // Chunk header düşülmüş MTU
        
        void update(int new_base) {
            base_mtu = new_base;
            rtp_mtu = base_mtu - RTP_HEADER_SIZE;
            chunk_mtu = rtp_mtu - sizeof(ChunkHeader);
        }

        friend std::ostream& operator<<(std::ostream& os, const MTUConfig& cfg) {
            return os << cfg.base_mtu;
        }
    } mtu;
    
    int jitter_buf_ms = 500;     // Jitter buffer latency
    int socket_buf_kb = 4096;    // Soket buffer boyutu (KB)
    int bitrate_kbps = 5000;     // Hedef bitrate
    
    // Profil skorlama (düşük = iyi)
    double score() const {
        UDPStats& stats = g_udp_stats; // global UDP istatistikleri
        double loss_penalty = stats.packet_loss * 100.0;
        double burst_penalty = stats.burst_loss * 200.0;
        double jitter_penalty = stats.jitter_ms * 0.5;
        double bw_penalty = (stats.bandwidth_mbps > 0) 
            ? std::max(0.0, (double)bitrate_kbps/1000.0 - stats.bandwidth_mbps) * 10.0
            : 0.0;
        return loss_penalty + burst_penalty + jitter_penalty + bw_penalty;
    }
    
    // MTU optimizasyonu
    void optimize_mtu() {
        UDPStats& stats = g_udp_stats;
        
        // Baz MTU seçimi (bitrate'e göre)
        int target_mtu = 1200;
        if (bitrate_kbps <= 6000)  target_mtu = 800;
        else if (bitrate_kbps <= 9000)  target_mtu = 960;
        else if (bitrate_kbps <= 12000) target_mtu = 1000;
        
        // Koşullara göre ayarla
        if (stats.packet_loss > 0.15 || stats.burst_loss > 0.05) {
            target_mtu = std::min(target_mtu, 800); // Yüksek kayıpta küçült
        }
        if (stats.jitter_ms > 50) {
            target_mtu = std::min(target_mtu, 900); // Yüksek jitter'da küçült
        }
        
        // Kademeli değişim (ani değişimlerden kaçın)
        int diff = target_mtu - mtu.base_mtu;
        if (std::abs(diff) >= 100) {
            int new_mtu = mtu.base_mtu + ((diff > 0) ? 100 : -100);
            mtu.update(new_mtu);
            
            // RTP ve Chunk MTU'ları güncelle
            if (g_rtp_pay) {
                set_int_async(G_OBJECT(g_rtp_pay), "mtu", mtu.rtp_mtu);
            }
        }
    }
    
    // Jitter buffer optimizasyonu
    void optimize_jitter_buffer() {
        UDPStats& stats = g_udp_stats;
        
        // RTT ve jitter bazlı hedef hesapla
        int target_latency = (int)(stats.avg_rtt_ms * 1.5 + stats.jitter_ms * 2.0);
        target_latency = std::max(100, std::min(2000, target_latency));
        
        // Burst kayıp varsa buffer'ı arttır
        if (stats.burst_loss > 0.02) {
            target_latency = (int)(target_latency * 1.2);
        }
        
        // Kademeli değişim
        int diff = target_latency - jitter_buf_ms;
        if (std::abs(diff) >= 50) {
            jitter_buf_ms += (diff > 0) ? 50 : -50;
        }
    }
    
    // Soket buffer optimizasyonu
    void optimize_socket_buffer() {
        UDPStats& stats = g_udp_stats;
        
        // Baz buffer boyutu (bitrate'e göre)
        int base_kb = 4096;
        if (bitrate_kbps > 10000) base_kb = 8192;
        if (bitrate_kbps > 20000) base_kb = 16384;
        
        // Jitter ve burst kayıp varsa arttır
        if (stats.jitter_ms > 30 || stats.burst_loss > 0.02) {
            base_kb = (int)(base_kb * 1.5);
        }
        
        // Kademeli değişim
        int diff = base_kb - socket_buf_kb;
        if (std::abs(diff) >= 1024) {
            socket_buf_kb += (diff > 0) ? 1024 : -1024;
        }
    }
    
    // Bitrate optimizasyonu
    void optimize_bitrate(int max_kbps) {
        UDPStats& stats = g_udp_stats;
        
        // Bant genişliği bazlı hedef
        int target = (stats.bandwidth_mbps > 0)
            ? (int)(stats.bandwidth_mbps * 1000.0 * 0.8) // %80 headroom
            : bitrate_kbps;
            
        // Kayıp ve jitter durumunda düşür
        if (stats.packet_loss > 0.1) {
            target = (int)(target * 0.8);
        }
        if (stats.jitter_ms > 50) {
            target = (int)(target * 0.9);
        }
        
        // Sınırlar
        target = std::max(2000, std::min(max_kbps, target));
        
        // Kademeli değişim
        int diff = target - bitrate_kbps;
        if (std::abs(diff) >= 500) {
            bitrate_kbps += (diff > 0) ? 500 : -500;
        }
    }
};

static NetworkProfile g_net_profile;

// peer RX & RTT'ye göre bitrate/MTU adaptasyonu
static void adaptive_net_loop(GstElement* enc, GstElement* pay, int start_kbps) {
    while (!g_stop) {
        // Tüm parametreleri optimize et
        g_net_profile.optimize_mtu();
        g_net_profile.optimize_jitter_buffer();
        g_net_profile.optimize_socket_buffer();
        g_net_profile.optimize_bitrate(start_kbps);
        
        // Değişiklikleri uygula
        set_encoder_bitrate(enc, g_net_profile.bitrate_kbps);
        set_int_async(G_OBJECT(pay), "mtu", g_net_profile.mtu.base_mtu);
        
        // Socket buffer güncelle
        int buf_bytes = g_net_profile.socket_buf_kb * 1024;
        if (g_shared_socket) {
            g_socket_set_option(g_shared_socket->get(), SOL_SOCKET, SO_SNDBUF, buf_bytes, nullptr);
            g_socket_set_option(g_shared_socket->get(), SOL_SOCKET, SO_RCVBUF, buf_bytes, nullptr);
        }
        
        // Log
        std::cout << "[adapt] bitrate=" << g_net_profile.bitrate_kbps
                  << " kbps, mtu=" << g_net_profile.mtu.base_mtu
                  << ", jitter_buf=" << g_net_profile.jitter_buf_ms
                  << " ms, sock_buf=" << g_net_profile.socket_buf_kb
                  << " KB (score=" << g_net_profile.score() << ")\n";
                  
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Zero-copy GPU memory yönetimi
struct GPUMemoryPool {
    std::vector<GstBuffer*> free_buffers;
    std::mutex mtx;
    
    GstBuffer* acquire() {
        std::lock_guard<std::mutex> lock(mtx);
        if (free_buffers.empty()) {
            // Yeni GPU buffer oluştur
            GstBuffer* buf = gst_buffer_new_allocate(nullptr, 1024*1024, nullptr);
            if (!buf) return nullptr;
            
            // GPU memory'de alloc
            GstMapInfo map;
            if (!gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
                gst_buffer_unref(buf);
                return nullptr;
            }
            gst_buffer_unmap(buf, &map);
            
            // DMA-capable olarak işaretle
            GST_MINI_OBJECT_FLAG_SET(buf, GST_BUFFER_FLAG_ZERO_COPY);
            return buf;
        }
        
        GstBuffer* buf = free_buffers.back();
        free_buffers.pop_back();
        return buf;
    }
    
    void release(GstBuffer* buf) {
        if (!buf) return;
        std::lock_guard<std::mutex> lock(mtx);
        free_buffers.push_back(buf);
    }
    
    ~GPUMemoryPool() {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto buf : free_buffers) {
            gst_buffer_unref(buf);
        }
        free_buffers.clear();
    }
};

static GPUMemoryPool g_gpu_pool;

// Zero-copy buffer callback'leri
static void on_need_data(GstElement* /*appsrc*/, guint /*size*/, gpointer user_data) {
    auto* buf = g_gpu_pool.acquire();
    if (buf) {
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(user_data), buf);
        if (ret != GST_FLOW_OK) {
            g_gpu_pool.release(buf);
        }
    }
}

static void on_enough_data(GstElement* /*appsrc*/, gpointer /*user_data*/) {
    // Buffer'ı geri havuza koy
}

// =================== SENDER ===================
static GstElement* build_sender(const Args& a, GSocket* shared_sock,
                                GstElement** out_enc, GstElement** out_pay) {
  std::string enc_name = choose_h264_encoder();
  g_last_encoder_name = enc_name;
  std::cerr << "[nova] encoder: " << enc_name << std::endl;

  GstElement* pipe = gst_pipeline_new("sender");

  // GPU memory path için source seçimi
  GstElement* src = nullptr;
  GstElement* upload = nullptr;
  
  // NVIDIA GPU varsa
  if (has_nvidia_gpu()) {
    // nvarguscamerasrc veya v4l2src + nvvidconv
    if (g_str_has_prefix(a.device.c_str(), "/dev/video")) {
      src = gst_element_factory_make("v4l2src", "src");
      CHECK_ELEM(src, "v4l2src");
      set_str(src, "device", a.device);
      set_int(src, "io-mode", 4); // MMAP için
      
      upload = gst_element_factory_make("nvvidconv", "upload");
      CHECK_ELEM(upload, "nvvidconv");
      g_object_set(G_OBJECT(upload), 
        "nvbuf-memory-type", 3, // NVBUF_MEM_CUDA_UNIFIED
        NULL);
    } else {
      src = gst_element_factory_make("nvarguscamerasrc", "src");
      CHECK_ELEM(src, "nvarguscamerasrc");
    }
  }
  // VAAPI GPU varsa
  else if (has_vaapi_gpu()) {
    src = gst_element_factory_make("v4l2src", "src");
    CHECK_ELEM(src, "v4l2src");
    set_str(src, "device", a.device);
    set_int(src, "io-mode", 4); // MMAP için
    
    upload = gst_element_factory_make("vaapih264enc", "upload");
    CHECK_ELEM(upload, "vaapih264enc");
    g_object_set(G_OBJECT(upload),
      "low-latency", 1,
      "init-qp", 26,
      NULL);
  }
  // QSV GPU varsa
  else if (has_qsv_gpu()) {
    src = gst_element_factory_make("v4l2src", "src");
    CHECK_ELEM(src, "v4l2src");
    set_str(src, "device", a.device);
    set_int(src, "io-mode", 4); // MMAP için
    
    upload = gst_element_factory_make("qsvh264enc", "upload");
    CHECK_ELEM(upload, "qsvh264enc");
    g_object_set(G_OBJECT(upload),
      "low-latency", 1,
      "target-usage", 1, // En hızlı mod
      NULL);
  }
  // Fallback: Normal v4l2src
  else {
    src = gst_element_factory_make("v4l2src", "src");
    CHECK_ELEM(src, "v4l2src");
    set_str(src, "device", a.device);
  }

  GstElement* capsf = gst_element_factory_make("capsfilter", "caps_src");
  CHECK_ELEM(capsf, "capsfilter");

  GstCaps* caps = nullptr;
  GstElement* jpegdec = nullptr;
  
  // GPU memory-aware converter
  GstElement* conv = nullptr;
  if (has_nvidia_gpu()) {
    conv = gst_element_factory_make("nvvidconv", "conv");
    CHECK_ELEM(conv, "nvvidconv");
  } else if (has_vaapi_gpu()) {
    conv = gst_element_factory_make("vaapipostproc", "conv");
    CHECK_ELEM(conv, "vaapipostproc");
  } else {
    conv = gst_element_factory_make("videoconvert", "conv");
    CHECK_ELEM(conv, "videoconvert");
  }

  // GPU-specific caps ve pipeline
  if (has_nvidia_gpu()) {
    if (a.prefer_mjpg) {
      caps = gst_caps_new_simple("image/jpeg",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      jpegdec = gst_element_factory_make("nvjpegdec", "jpegdec");
      CHECK_ELEM(jpegdec, "nvjpegdec");

      gst_bin_add_many(GST_BIN(pipe), src, capsf, jpegdec, upload, conv, NULL);
      if (!gst_element_link_many(src, capsf, jpegdec, upload, conv, NULL)) {
        std::cerr << "Link failed (nvidia jpeg path)\n"; return nullptr;
      }
    } else {
      caps = gst_caps_new_simple("video/x-raw",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      gst_bin_add_many(GST_BIN(pipe), src, capsf, upload, conv, NULL);
      if (!gst_element_link_many(src, capsf, upload, conv, NULL)) {
        std::cerr << "Link failed (nvidia raw path)\n"; return nullptr;
      }
    }
  }
  // VAAPI path
  else if (has_vaapi_gpu()) {
    if (a.prefer_mjpg) {
      caps = gst_caps_new_simple("image/jpeg",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      jpegdec = gst_element_factory_make("vaapijpegdec", "jpegdec");
      CHECK_ELEM(jpegdec, "vaapijpegdec");

      gst_bin_add_many(GST_BIN(pipe), src, capsf, jpegdec, upload, conv, NULL);
      if (!gst_element_link_many(src, capsf, jpegdec, upload, conv, NULL)) {
        std::cerr << "Link failed (vaapi jpeg path)\n"; return nullptr;
      }
    } else {
      caps = gst_caps_new_simple("video/x-raw",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      gst_bin_add_many(GST_BIN(pipe), src, capsf, upload, conv, NULL);
      if (!gst_element_link_many(src, capsf, upload, conv, NULL)) {
        std::cerr << "Link failed (vaapi raw path)\n"; return nullptr;
      }
    }
  }
  // QSV path
  else if (has_qsv_gpu()) {
    if (a.prefer_mjpg) {
      caps = gst_caps_new_simple("image/jpeg",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      jpegdec = gst_element_factory_make("qsvjpegdec", "jpegdec");
      CHECK_ELEM(jpegdec, "qsvjpegdec");

      gst_bin_add_many(GST_BIN(pipe), src, capsf, jpegdec, upload, conv, NULL);
      if (!gst_element_link_many(src, capsf, jpegdec, upload, conv, NULL)) {
        std::cerr << "Link failed (qsv jpeg path)\n"; return nullptr;
      }
    } else {
      caps = gst_caps_new_simple("video/x-raw",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      gst_bin_add_many(GST_BIN(pipe), src, capsf, upload, conv, NULL);
      if (!gst_element_link_many(src, capsf, upload, conv, NULL)) {
        std::cerr << "Link failed (qsv raw path)\n"; return nullptr;
      }
    }
  }
  // CPU fallback path
  else {
    if (a.prefer_mjpg) {
      caps = gst_caps_new_simple("image/jpeg",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      jpegdec = gst_element_factory_make("jpegdec", "jpegdec");
      CHECK_ELEM(jpegdec, "jpegdec");

      gst_bin_add_many(GST_BIN(pipe), src, capsf, jpegdec, conv, NULL);
      if (!gst_element_link_many(src, capsf, jpegdec, conv, NULL)) {
        std::cerr << "Link failed (cpu jpeg path)\n"; return nullptr;
      }
    } else {
      caps = gst_caps_new_simple("video/x-raw",
        "width",  G_TYPE_INT, a.width,
        "height", G_TYPE_INT, a.height,
        "framerate", GST_TYPE_FRACTION, a.fps, 1, NULL);
      g_object_set(G_OBJECT(capsf), "caps", caps, NULL);
      gst_caps_unref(caps);

      gst_bin_add_many(GST_BIN(pipe), src, capsf, conv, NULL);
      if (!gst_element_link_many(src, capsf, conv, NULL)) {
        std::cerr << "Link failed (cpu raw path)\n"; return nullptr;
      }
    }
  }

  GstElement *tee = gst_element_factory_make("tee", "tee");
  CHECK_ELEM(tee, "tee");
  gst_bin_add(GST_BIN(pipe), tee);
  if (!gst_element_link(conv, tee)) { std::cerr << "link fail conv->tee\n"; return nullptr; }

  // preview branch
  GstElement *qprev  = gst_element_factory_make("queue", "qprev");  CHECK_ELEM(qprev, "queue");
  GstElement *conv2  = gst_element_factory_make("videoconvert", "conv2"); CHECK_ELEM(conv2, "videoconvert");
  GstElement *flip2  = gst_element_factory_make("videoflip", "flip2"); CHECK_ELEM(flip2, "videoflip");
  set_arg(flip2, "method", "horizontal-flip");
  GstElement *sink2  = gst_element_factory_make(headless_env() ? "fakesink" : "autovideosink", "local_preview"); CHECK_ELEM(sink2, headless_env() ? "fakesink" : "autovideosink");
  set_bool(sink2, "sync", TRUE);

  gst_bin_add_many(GST_BIN(pipe), qprev, conv2, flip2, sink2, NULL);
  if (!gst_element_link_many(tee, qprev, conv2, flip2, sink2, NULL)) {
    std::cerr << "preview link fail\n"; return nullptr;
  }

  // network branch
  GstElement *q1 = gst_element_factory_make("queue", "q1"); CHECK_ELEM(q1, "queue");
  set_int(q1, "max-size-time", 0); set_int(q1, "max-size-buffers", 0); set_int(q1, "max-size-bytes", 0); set_int(q1, "leaky", 2);

  GstElement *enc = gst_element_factory_make(enc_name.c_str(), "enc"); CHECK_ELEM(enc, enc_name.c_str());
  if (enc_name == "nvh264enc") {
    set_str(enc, "preset", "low-latency-hq"); set_str(enc, "rc", "cbr");
    set_int(enc, "bitrate", a.bitrate_kbps); set_int(enc, "key-int-max", a.keyint); set_bool(enc, "zerolatency", TRUE);
  } else if (enc_name == "vaapih264enc") {
    set_arg(enc, "rate-control", "cbr");
    set_int(enc, "bitrate", a.bitrate_kbps);
    set_int(enc, "keyframe-period", a.keyint);
  } else if (enc_name == "qsvh264enc") {
    set_str(enc, "rate-control", "cbr"); set_int(enc, "bitrate", a.bitrate_kbps*1000); set_int(enc, "gop-size", a.keyint);
  } else if (enc_name == "vah264enc") {
    set_int(enc, "bitrate", a.bitrate_kbps*1000);
  } else { // x264enc
    set_arg(enc, "tune", "zerolatency"); set_arg(enc, "speed-preset", "ultrafast");
    set_int(enc, "bitrate", a.bitrate_kbps); set_int(enc, "key-int-max", a.keyint); set_bool(enc, "byte-stream", TRUE);
  }

  GstElement *parse = gst_element_factory_make("h264parse", "parse"); CHECK_ELEM(parse, "h264parse");
  set_int(parse, "config-interval", 1);
  set_arg(parse, "stream-format", "byte-stream");
  set_arg(parse, "alignment", "au");

  GstElement *pay = gst_element_factory_make("rtph264pay", "pay"); CHECK_ELEM(pay, "rtph264pay");
  set_int(pay, "pt", 96); set_int(pay, "mtu", a.mtu); set_int(pay, "config-interval", 1);

  GstElement *sink = gst_element_factory_make("udpsink", "udpsink"); CHECK_ELEM(sink, "udpsink");
  set_str(sink, "host", a.peer_ip);
  set_int(sink, "port", a.media_port);
  set_bool(sink, "sync", FALSE); set_bool(sink, "async", FALSE);
  g_object_set(G_OBJECT(sink), "socket", g_object_ref(shared_sock), NULL);

  gst_bin_add_many(GST_BIN(pipe), q1, enc, parse, pay, sink, NULL);
  if (!gst_element_link_many(tee, q1, enc, parse, pay, sink, NULL)) return nullptr;

  // bus watch
  GstBus* bus = gst_element_get_bus(pipe);
  gst_bus_add_watch(bus, bus_cb, (gpointer)"sender");
  gst_object_unref(bus);

  if (out_enc) *out_enc = enc;
  if (out_pay) *out_pay = pay;
  return pipe;
}

// =================== RECEIVER ===================
static GstElement* build_receiver(const Args& a, GSocket* shared_sock,
                                  GstElement** out_jbuf, RateMeter** out_rxmeter) {
  GstElement* pipe = gst_pipeline_new("receiver");

  auto src = gst_element_factory_make("udpsrc", "udpsrc");
  CHECK_ELEM(src, "udpsrc");
  g_object_set(G_OBJECT(src), "socket", g_object_ref(shared_sock), NULL);

  // RTP caps'i DOĞRUDAN udpsrc'ye veriyoruz (clock-rate dahil)
  GstCaps* rtp_caps = (!a.use_ts)
    ? gst_caps_new_simple("application/x-rtp",
        "media",         G_TYPE_STRING, "video",
        "encoding-name", G_TYPE_STRING, "H264",
        "payload",       G_TYPE_INT,    96,
        "clock-rate",    G_TYPE_INT,    90000, NULL)
    : gst_caps_new_simple("application/x-rtp",
        "media",         G_TYPE_STRING, "video",
        "encoding-name", G_TYPE_STRING, "MP2T",
        "payload",       G_TYPE_INT,    33,
        "clock-rate",    G_TYPE_INT,    90000, NULL);
  g_object_set(G_OBJECT(src), "caps", rtp_caps, NULL);
  gst_caps_unref(rtp_caps);

  auto jbuf = gst_element_factory_make("rtpjitterbuffer", "jbuf");
  CHECK_ELEM(jbuf, "rtpjitterbuffer");
  set_int(jbuf, "latency", a.latency_ms);
  // bazı sürümlerde yok; varsa set edelim
  set_bool_if_exists(G_OBJECT(jbuf), "drop-on-late", TRUE);
  // 'mode' gibi hatalı/uyumsuz property yok; sadece geçerli olanları set ediyoruz.

  auto depay = (!a.use_ts)
    ? gst_element_factory_make("rtph264depay", "depay")
    : gst_element_factory_make("rtpmp2tdepay", "depay");
  CHECK_ELEM(depay, "depay");

  // RX hız ölçer
  auto rxid = gst_element_factory_make("identity", "rxmeter");
  CHECK_ELEM(rxid, "identity");
  g_object_set(G_OBJECT(rxid), "signal-handoffs", TRUE, NULL);
  static RateMeter* rxmeter = new RateMeter();
  g_signal_connect(rxid, "handoff", G_CALLBACK(on_handoff_meter), rxmeter);

  auto parse = gst_element_factory_make("h264parse", "parse");
  CHECK_ELEM(parse, "h264parse");

  // Decoder (GPU varsa tercih et)
  GstElement* dec = gst_element_factory_make("nvh264dec", "dec");
  if (!dec) dec = gst_element_factory_make("vah264dec", "dec");
  if (!dec) dec = gst_element_factory_make("qsvh264dec", "dec");
  if (!dec) dec = gst_element_factory_make("avdec_h264", "dec");
  CHECK_ELEM(dec, "h264 decoder");

  auto conv = gst_element_factory_make("videoconvert", "conv");
  CHECK_ELEM(conv, "videoconvert");

  auto flip = gst_element_factory_make("videoflip", "flip");
  CHECK_ELEM(flip, "videoflip");
  set_arg(flip, "method", "horizontal-flip");

  auto sink = gst_element_factory_make(headless_env() ? "fakesink" : "autovideosink", "sink");
  CHECK_ELEM(sink, headless_env() ? "fakesink" : "autovideosink");
  set_bool(sink, "sync", TRUE);

  if (!a.use_ts) {
    gst_bin_add_many(GST_BIN(pipe), src, jbuf, depay, rxid, parse, dec, conv, flip, sink, NULL);
    if (!gst_element_link_many(src, jbuf, depay, rxid, parse, dec, conv, flip, sink, NULL)) return nullptr;
  } else {
    auto tsdemux = gst_element_factory_make("tsdemux", "tsdemux");
    CHECK_ELEM(tsdemux, "tsdemux");
    gst_bin_add_many(GST_BIN(pipe), src, jbuf, depay, tsdemux, rxid, parse, dec, conv, flip, sink, NULL);
    if (!gst_element_link_many(src, jbuf, depay, tsdemux, NULL)) return nullptr;
    g_signal_connect(tsdemux, "pad-added",
      G_CALLBACK(+[] (GstElement* /*demux*/, GstPad* newpad, gpointer user_data){
        auto parse = static_cast<GstElement*>(user_data);
        GstPad* sinkpad = gst_element_get_static_pad(parse, "sink");
        if (!gst_pad_is_linked(sinkpad)) gst_pad_link(newpad, sinkpad);
        gst_object_unref(sinkpad);
      }), parse);
    if (!gst_element_link_many(rxid, parse, dec, conv, flip, sink, NULL)) return nullptr;
  }

  // bus watch
  GstBus* bus = gst_element_get_bus(pipe);
  gst_bus_add_watch(bus, bus_cb, (gpointer)"receiver");
  gst_object_unref(bus);

  if (out_jbuf) *out_jbuf = jbuf;
  if (out_rxmeter) *out_rxmeter = rxmeter;
  return pipe;
}

// =================== MAIN ===================
int main(int argc, char** argv) {
  ensure_runtime_env(); // XDG_RUNTIME_DIR yoksa set et
  gst_init(&argc, &argv);
  std::signal(SIGINT, sig_handler);
  std::signal(SIGTERM, sig_handler);

  if (argc < 4) {
    std::cerr << "Kullanım: ./nova_engine <peer_ip> <media_port> <ctrl_port>\n";
    return 1;
  }

  Args a;
  a.peer_ip    = argv[1];
  a.media_port = std::stoi(argv[2]);
  a.ctrl_port  = std::stoi(argv[3]);

  if (!auto_select_best_camera(a)) {
    std::cerr << "Kamera bulunamadı veya caps doğrulanamadı.\n";
    return 1;
  }

  std::cout << "[auto] device=" << a.device
            << " mode=" << (a.prefer_mjpg ? "MJPG" : "RAW")
            << " " << a.width << "x" << a.height
            << "@" << a.fps << " selected\n";

  // TEK MEDYA PORTU
  GSocket* media_sock = create_shared_udp_socket(a.media_port);
  if (!media_sock) {
    std::cerr << "Medya soketi oluşturulamadı.\n";
    return 1;
  }

  ControlChannel ctrl(a.peer_ip, a.ctrl_port);
  if (!ctrl.start()) { std::cerr << "Control channel start failed\n"; g_object_unref(media_sock); return 1; }

  GstElement *enc=nullptr, *pay=nullptr, *jbuf=nullptr;
  RateMeter* rxmeter=nullptr;

  auto sender   = build_sender(a, media_sock, &enc, &pay);
  auto receiver = build_receiver(a, media_sock, &jbuf, &rxmeter);
  if (!sender || !receiver || !enc || !pay || !jbuf || !rxmeter) {
    ctrl.stop();
    if (sender) gst_object_unref(sender);
    if (receiver) gst_object_unref(receiver);
    g_object_unref(media_sock);
    return 1;
  }

  // Adaptasyon thread'leri
  std::thread jadapt_thr([&]{ adaptive_jbuf_loop(jbuf); });
  std::thread rx_stat_thr([&]{ rate_meter_loop(rxmeter, &g_local_rx_kbps); });
  std::thread net_adapt_thr([&]{ adaptive_net_loop(enc, pay, a.bitrate_kbps); });
  std::thread sock_monitor_thr(socket_monitor_thread);

  gst_element_set_state(receiver, GST_STATE_PLAYING);
  gst_element_set_state(sender,   GST_STATE_PLAYING);

  // ---- GLib main loop + STDIN watcher (ESC/q) ----
  g_loop = g_main_loop_new(NULL, FALSE);
  GIOChannel* ch = g_io_channel_unix_new(STDIN_FILENO);
  g_io_channel_set_encoding(ch, NULL, NULL);
  g_io_channel_set_flags(ch, (GIOFlags)(g_io_channel_get_flags(ch) | G_IO_FLAG_NONBLOCK), NULL);
  g_io_add_watch(ch, (GIOCondition)(G_IO_IN | G_IO_HUP | G_IO_ERR | G_IO_NVAL), stdin_cb, NULL);

  g_main_loop_run(g_loop);

  // ---- shutdown ----
  g_stop = true;
  if (net_adapt_thr.joinable()) net_adapt_thr.join();
  if (rx_stat_thr.joinable())   rx_stat_thr.join();
  if (jadapt_thr.joinable())    jadapt_thr.join();
  if (sock_monitor_thr.joinable()) sock_monitor_thr.join();

  gst_element_set_state(sender,   GST_STATE_NULL);
  gst_element_set_state(receiver, GST_STATE_NULL);
  gst_object_unref(sender);
  gst_object_unref(receiver);
  ctrl.stop();

  if (ch) g_io_channel_unref(ch);
  if (g_loop) { g_main_loop_unref(g_loop); g_loop=nullptr; }
  if (media_sock) g_object_unref(media_sock);
  return 0;
}
