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

// Linux UDP (kontrol kanalı - raw)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// GLib/GIO
#include <gio/gio.h>

static std::atomic<bool> g_stop(false);
static GMainLoop* g_loop = nullptr;

static void sig_handler(int){
  g_stop = true;
  if (g_loop) g_main_loop_quit(g_loop);
}

struct Args {
  std::string peer_ip;

  // === YENİ: tek port tasarımı ===
  int media_port; // hem send hem recv (RTP)
  int ctrl_port;  // hem send hem recv (PING/PONG)

  bool use_ts = false;
  int  mtu = 1200;
  int  bitrate_kbps = 18000;
  int  keyint = 60;
  int  latency_ms = 200;

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

// ---- ortak UDP GSocket (tek port) oluşturucu ----
static GSocket* create_shared_udp_socket(int port) {
  GError* err = nullptr;
  GSocket* s = g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM, G_SOCKET_PROTOCOL_UDP, &err);
  if (!s) {
    std::cerr << "[udp] g_socket_new failed: " << (err? err->message : "") << "\n";
    if (err) g_error_free(err);
    return nullptr;
  }

  // REUSEADDR/REUSEPORT → aynı portu hem send hem recv için kullanabilmek adına
  g_socket_set_option(s, SOL_SOCKET, SO_REUSEADDR, 1, nullptr);
#ifdef SO_REUSEPORT
  g_socket_set_option(s, SOL_SOCKET, SO_REUSEPORT, 1, nullptr);
#endif

  GInetAddress* any = g_inet_address_new_any(G_SOCKET_FAMILY_IPV4);
  GSocketAddress* sa = g_inet_socket_address_new(any, port);
  g_object_unref(any);

  if (!g_socket_bind(s, sa, TRUE, &err)) {
    std::cerr << "[udp] bind " << port << " failed: " << (err? err->message : "") << "\n";
    if (err) g_error_free(err);
    g_object_unref(sa);
    g_object_unref(s);
    return nullptr;
  }
  g_object_unref(sa);
  return s;
}

// ---- kontrol kanalı (PING/PONG) — tek soket, tek port ----
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
    while (!g_stop) {
      long long now = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
      char buf[64];
      int n = snprintf(buf, sizeof(buf), "PING %lld", now);
      sendto(fd_, buf, n, 0, (sockaddr*)&peer_addr_, sizeof(peer_addr_));
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
        std::cout << "[ctrl] RTT ~ " << (now - t0) << " ms\n";
      }
    }
  }

  std::string peer_ip_;
  int port_;
  int fd_ = -1;
  sockaddr_in peer_addr_{};
  std::thread send_thr_, recv_thr_;
};

// ---- GStreamer Bus watcher (ERROR/EOS -> quit) ----
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

// ---- STDIN watcher (ESC/q -> quit) ----
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
  return TRUE; // watch devam
}

// ---- sender (tek port için paylaşılan socket kullanır) ----
static GstElement* build_sender(const Args& a, GSocket* shared_sock) {
  std::string enc_name = choose_h264_encoder();
  std::cerr << "[nova] encoder: " << enc_name << std::endl;

  GstElement* pipe = gst_pipeline_new("sender");

  GstElement* src = gst_element_factory_make("v4l2src", "src");
  CHECK_ELEM(src, "v4l2src");
  set_str(src, "device", a.device);

  GstElement* capsf = gst_element_factory_make("capsfilter", "caps_src");
  CHECK_ELEM(capsf, "capsfilter");

  GstCaps* caps = nullptr;
  GstElement* jpegdec = nullptr;
  GstElement* conv = gst_element_factory_make("videoconvert", "conv");
  CHECK_ELEM(conv, "videoconvert");

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
      std::cerr << "Link failed (src->jpegdec->conv)\n"; return nullptr;
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
      std::cerr << "Link failed (src->conv)\n"; return nullptr;
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
  GstElement *sink2  = gst_element_factory_make("autovideosink", "local_preview"); CHECK_ELEM(sink2, "autovideosink");
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
  // hedef: karşı tarafın media_port'u
  set_str(sink, "host", a.peer_ip);
  set_int(sink, "port", a.media_port);
  set_bool(sink, "sync", FALSE); set_bool(sink, "async", FALSE);

  // === KRİTİK: aynı porttan gönderim için aynı GSocket'i kullan ===
  g_object_set(G_OBJECT(sink), "socket", g_object_ref(shared_sock), NULL);

  gst_bin_add_many(GST_BIN(pipe), q1, enc, parse, pay, sink, NULL);
  if (!gst_element_link_many(tee, q1, enc, parse, pay, sink, NULL)) return nullptr;

  // bus watch
  GstBus* bus = gst_element_get_bus(pipe);
  gst_bus_add_watch(bus, bus_cb, (gpointer)"sender");
  gst_object_unref(bus);

  return pipe;
}

// ---- receiver (tek port için paylaşılan socket kullanır) ----
static GstElement* build_receiver(const Args& a, GSocket* shared_sock) {
  GstElement* pipe = gst_pipeline_new("receiver");

  auto src = gst_element_factory_make("udpsrc", "udpsrc");
  CHECK_ELEM(src, "udpsrc");

  // === KRİTİK: udpsrc de aynı GSocket'i kullanıyor (aynı port) ===
  g_object_set(G_OBJECT(src), "socket", g_object_ref(shared_sock), NULL);

  // Not: "port" vermiyoruz; socket zaten bağlı (bind) durumda.

  auto capf = gst_element_factory_make("capsfilter", "capf");
  CHECK_ELEM(capf, "capsfilter");
  GstCaps* caps = (!a.use_ts)
    ? gst_caps_new_simple("application/x-rtp",
        "media", G_TYPE_STRING, "video",
        "encoding-name", G_TYPE_STRING, "H264",
        "payload", G_TYPE_INT, 96, NULL)
    : gst_caps_new_simple("application/x-rtp",
        "media", G_TYPE_STRING, "video",
        "encoding-name", G_TYPE_STRING, "MP2T",
        "payload", G_TYPE_INT, 33, NULL);
  g_object_set(G_OBJECT(capf), "caps", caps, NULL);
  gst_caps_unref(caps);

  auto jbuf = gst_element_factory_make("rtpjitterbuffer", "jbuf");
  CHECK_ELEM(jbuf, "rtpjitterbuffer");
  set_int(jbuf, "latency", a.latency_ms);
  set_bool(jbuf, "mode", TRUE);

  auto depay = (!a.use_ts)
    ? gst_element_factory_make("rtph264depay", "depay")
    : gst_element_factory_make("rtpmp2tdepay", "depay");
  CHECK_ELEM(depay, "depay");

  auto parse = gst_element_factory_make("h264parse", "parse");
  CHECK_ELEM(parse, "h264parse");
  auto dec = gst_element_factory_make("avdec_h264", "dec");
  CHECK_ELEM(dec, "avdec_h264");
  auto conv = gst_element_factory_make("videoconvert", "conv");
  CHECK_ELEM(conv, "videoconvert");

  auto flip = gst_element_factory_make("videoflip", "flip");
  CHECK_ELEM(flip, "videoflip");
  set_arg(flip, "method", "horizontal-flip");

  auto sink = gst_element_factory_make("autovideosink", "sink");
  CHECK_ELEM(sink, "autovideosink");
  set_bool(sink, "sync", TRUE);

  if (!a.use_ts) {
    gst_bin_add_many(GST_BIN(pipe), src, capf, jbuf, depay, parse, dec, conv, flip, sink, NULL);
    if (!gst_element_link_many(src, capf, jbuf, depay, parse, dec, conv, flip, sink, NULL)) return nullptr;
  } else {
    auto tsdemux = gst_element_factory_make("tsdemux", "tsdemux");
    CHECK_ELEM(tsdemux, "tsdemux");
    gst_bin_add_many(GST_BIN(pipe), src, capf, jbuf, depay, tsdemux, parse, dec, conv, flip, sink, NULL);
    if (!gst_element_link_many(src, capf, jbuf, depay, tsdemux, NULL)) return nullptr;
    g_signal_connect(tsdemux, "pad-added",
      G_CALLBACK(+[] (GstElement* /*demux*/, GstPad* newpad, gpointer user_data){
        auto parse = static_cast<GstElement*>(user_data);
        GstPad* sinkpad = gst_element_get_static_pad(parse, "sink");
        if (!gst_pad_is_linked(sinkpad)) gst_pad_link(newpad, sinkpad);
        gst_object_unref(sinkpad);
      }), parse);
    if (!gst_element_link_many(parse, dec, conv, flip, sink, NULL)) return nullptr;
  }

  // bus watch
  GstBus* bus = gst_element_get_bus(pipe);
  gst_bus_add_watch(bus, bus_cb, (gpointer)"receiver");
  gst_object_unref(bus);

  return pipe;
}

int main(int argc, char** argv) {
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

  // === TEK MEDYA PORTU İÇİN ORTAK GSOCKET ===
  GSocket* media_sock = create_shared_udp_socket(a.media_port);
  if (!media_sock) {
    std::cerr << "Medya soketi oluşturulamadı.\n";
    return 1;
  }

  ControlChannel ctrl(a.peer_ip, a.ctrl_port);
  if (!ctrl.start()) { std::cerr << "Control channel start failed\n"; g_object_unref(media_sock); return 1; }

  auto sender   = build_sender(a, media_sock);
  auto receiver = build_receiver(a, media_sock);
  if (!sender || !receiver) { ctrl.stop(); g_object_unref(media_sock); return 1; }

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
