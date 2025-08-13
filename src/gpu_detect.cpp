#include "gpu_detect.hpp"
#include <gst/gst.h>
#include <cstdlib>

static bool has_factory(const char* name) {
  GstElementFactory* f = gst_element_factory_find(name);
  if (f) { gst_object_unref(f); return true; }
  return false;
}

std::string choose_h264_encoder() {
  const char* force = std::getenv("NOVA_FORCE_X264");
  if (force && *force=='1') return "x264enc";
  if (has_factory("nvh264enc"))    return "nvh264enc";     // NVIDIA
  if (has_factory("vaapih264enc")) return "vaapih264enc";  // VAAPI
  if (has_factory("qsvh264enc"))   return "qsvh264enc";    // Intel QSV
  if (has_factory("vah264enc"))    return "vah264enc";     // AMD (bazı distrolar)
  return "x264enc"; // CPU fallback
}
