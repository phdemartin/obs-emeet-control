#include <obs/obs-module.h>
#include <obs/obs.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include "face_tracker.hpp"
#include "lut_processor.hpp"
#include "v4l2_helper.hpp"

OBS_DECLARE_MODULE()

MODULE_EXPORT const char *obs_module_description(void)
{
    return "EMEET SmartCam S800 & UVC Webcam Control Filter (V4L2 + Face Tracking + HDR + 3D LUTs)";
}

namespace {

struct EmeetFilterContext {
    obs_source_t *source = nullptr;
    std::mutex mtx;

    // Hardware V4L2
    std::string device_path;
    bool apply_hw = true;

    // Software effects
    bool face_tracking = false;
    double track_zoom = 1.5;
    double track_smooth = 0.08;
    emeet::HdrMode hdr_mode = emeet::HdrMode::Off;
    bool lut_enabled = false;
    std::string lut_path;
    bool flip_h = false;
    bool flip_v = false;

    emeet::FaceTracker tracker;
    emeet::Lut3D lut;

    // Persistent output frame and scratch Mats (avoid per-frame heap/TBB churn)
    struct obs_source_frame out_frame {};
    uint32_t out_w = 0;
    uint32_t out_h = 0;
    bool out_init = false;
    cv::Mat work_yuv;
    cv::Mat work_bgr;
};

void ensure_out_frame(EmeetFilterContext *ctx, uint32_t w, uint32_t h)
{
    if (ctx->out_init && ctx->out_w == w && ctx->out_h == h)
        return;
    if (ctx->out_init)
        obs_source_frame_free(&ctx->out_frame);
    std::memset(&ctx->out_frame, 0, sizeof(ctx->out_frame));
    obs_source_frame_init(&ctx->out_frame, VIDEO_FORMAT_BGRA, w, h);
    ctx->out_frame.full_range = true;
    ctx->out_w = w;
    ctx->out_h = h;
    ctx->out_init = true;
}

bool frame_to_bgr(const struct obs_source_frame *frame, cv::Mat &bgr, cv::Mat &yuv_scratch)
{
    const int w = static_cast<int>(frame->width);
    const int h = static_cast<int>(frame->height);

    switch (frame->format) {
    case VIDEO_FORMAT_YUY2: {
        cv::Mat src(h, w, CV_8UC2, frame->data[0], frame->linesize[0]);
        cv::cvtColor(src, bgr, cv::COLOR_YUV2BGR_YUY2);
        return true;
    }
    case VIDEO_FORMAT_UYVY: {
        cv::Mat src(h, w, CV_8UC2, frame->data[0], frame->linesize[0]);
        cv::cvtColor(src, bgr, cv::COLOR_YUV2BGR_UYVY);
        return true;
    }
    case VIDEO_FORMAT_NV12: {
        cv::Mat y_plane(h, w, CV_8UC1, frame->data[0], frame->linesize[0]);
        cv::Mat uv_plane(h / 2, w / 2, CV_8UC2, frame->data[1], frame->linesize[1]);
        cv::cvtColorTwoPlane(y_plane, uv_plane, bgr, cv::COLOR_YUV2BGR_NV12);
        return true;
    }
    case VIDEO_FORMAT_I420: {
        yuv_scratch.create(h + h / 2, w, CV_8UC1);
        uint8_t *dst = yuv_scratch.data;
        for (int r = 0; r < h; ++r) {
            std::memcpy(dst, frame->data[0] + r * frame->linesize[0], w);
            dst += w;
        }
        for (int r = 0; r < h / 2; ++r) {
            std::memcpy(dst, frame->data[1] + r * frame->linesize[1], w / 2);
            dst += w / 2;
        }
        for (int r = 0; r < h / 2; ++r) {
            std::memcpy(dst, frame->data[2] + r * frame->linesize[2], w / 2);
            dst += w / 2;
        }
        cv::cvtColor(yuv_scratch, bgr, cv::COLOR_YUV2BGR_I420);
        return true;
    }
    case VIDEO_FORMAT_BGRA:
    case VIDEO_FORMAT_BGRX: {
        cv::Mat src(h, w, CV_8UC4, frame->data[0], frame->linesize[0]);
        cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
        return true;
    }
    case VIDEO_FORMAT_RGBA: {
        cv::Mat src(h, w, CV_8UC4, frame->data[0], frame->linesize[0]);
        cv::cvtColor(src, bgr, cv::COLOR_RGBA2BGR);
        return true;
    }
    default:
        return false;
    }
}

void sync_settings_from_hardware(obs_data_t *settings, const std::string &dev)
{
    if (dev.empty())
        return;
    int32_t v = 0;
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_ZOOM_ABSOLUTE, v))
        obs_data_set_int(settings, "hw_zoom", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_FOCUS_AUTO, v))
        obs_data_set_bool(settings, "hw_focus_auto", v != 0);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_FOCUS_ABSOLUTE, v))
        obs_data_set_int(settings, "hw_focus_abs", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_EXPOSURE_AUTO, v))
        obs_data_set_int(settings, "hw_exp_auto", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_EXPOSURE_ABSOLUTE, v))
        obs_data_set_int(settings, "hw_exp_abs", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_AUTO_WHITE_BALANCE, v))
        obs_data_set_bool(settings, "hw_wb_auto", v != 0);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_WHITE_BALANCE_TEMPERATURE, v))
        obs_data_set_int(settings, "hw_wb_temp", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_BRIGHTNESS, v))
        obs_data_set_int(settings, "hw_brightness", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_CONTRAST, v))
        obs_data_set_int(settings, "hw_contrast", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_SATURATION, v))
        obs_data_set_int(settings, "hw_saturation", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_HUE, v))
        obs_data_set_int(settings, "hw_hue", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_SHARPNESS, v))
        obs_data_set_int(settings, "hw_sharpness", v);
    if (emeet::v4l2_get_ctrl(dev, V4L2_CID_POWER_LINE_FREQUENCY, v))
        obs_data_set_int(settings, "hw_powerline", v);
}

const char *emeet_get_name(void *)
{
    return "EMEET CamControl (V4L2 + Auto-Framing + HDR + LUT)";
}

void emeet_update(void *data, obs_data_t *settings)
{
    auto *ctx = static_cast<EmeetFilterContext *>(data);
    std::lock_guard<std::mutex> lock(ctx->mtx);

    const char *dev_c = obs_data_get_string(settings, "device_path");
    ctx->device_path = dev_c ? dev_c : "";
    ctx->apply_hw = obs_data_get_bool(settings, "apply_hw");

    bool new_track = obs_data_get_bool(settings, "face_tracking");
    if (new_track && !ctx->face_tracking)
        ctx->tracker.reset();
    ctx->face_tracking = new_track;
    ctx->track_zoom = obs_data_get_double(settings, "track_zoom");
    ctx->track_smooth = obs_data_get_double(settings, "track_smooth");

    ctx->hdr_mode = static_cast<emeet::HdrMode>(obs_data_get_int(settings, "hdr_mode"));
    ctx->lut_enabled = obs_data_get_bool(settings, "lut_enabled");
    const char *lut_c = obs_data_get_string(settings, "lut_file");
    if (!lut_c || !lut_c[0])
        lut_c = obs_data_get_string(settings, "lut_path");
    ctx->lut_path = lut_c ? lut_c : "";
    if (ctx->lut_enabled && !ctx->lut_path.empty())
        ctx->lut.load(ctx->lut_path);

    ctx->flip_h = obs_data_get_bool(settings, "flip_h");
    ctx->flip_v = obs_data_get_bool(settings, "flip_v");

    if (ctx->apply_hw && !ctx->device_path.empty()) {
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_ZOOM_ABSOLUTE,
                             static_cast<int32_t>(obs_data_get_int(settings, "hw_zoom")));

        bool focus_auto = obs_data_get_bool(settings, "hw_focus_auto");
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_FOCUS_AUTO, focus_auto ? 1 : 0);
        if (!focus_auto) {
            emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_FOCUS_ABSOLUTE,
                                 static_cast<int32_t>(obs_data_get_int(settings, "hw_focus_abs")));
        }

        int32_t exp_auto = static_cast<int32_t>(obs_data_get_int(settings, "hw_exp_auto"));
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_EXPOSURE_AUTO, exp_auto);
        if (exp_auto == 1) {
            emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_EXPOSURE_ABSOLUTE,
                                 static_cast<int32_t>(obs_data_get_int(settings, "hw_exp_abs")));
        }

        bool wb_auto = obs_data_get_bool(settings, "hw_wb_auto");
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_AUTO_WHITE_BALANCE, wb_auto ? 1 : 0);
        if (!wb_auto) {
            emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_WHITE_BALANCE_TEMPERATURE,
                                 static_cast<int32_t>(obs_data_get_int(settings, "hw_wb_temp")));
        }

        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_BRIGHTNESS,
                             static_cast<int32_t>(obs_data_get_int(settings, "hw_brightness")));
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_CONTRAST,
                             static_cast<int32_t>(obs_data_get_int(settings, "hw_contrast")));
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_SATURATION,
                             static_cast<int32_t>(obs_data_get_int(settings, "hw_saturation")));
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_HUE,
                             static_cast<int32_t>(obs_data_get_int(settings, "hw_hue")));
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_SHARPNESS,
                             static_cast<int32_t>(obs_data_get_int(settings, "hw_sharpness")));
        emeet::v4l2_set_ctrl(ctx->device_path, V4L2_CID_POWER_LINE_FREQUENCY,
                             static_cast<int32_t>(obs_data_get_int(settings, "hw_powerline")));
    }
}

void *emeet_create(obs_data_t *settings, obs_source_t *source)
{
    auto *ctx = new EmeetFilterContext();
    ctx->source = source;

    const char *dev_c = obs_data_get_string(settings, "device_path");
    if (!dev_c || !dev_c[0]) {
        auto devs = emeet::list_v4l2_devices();
        if (!devs.empty()) {
            obs_data_set_string(settings, "device_path", devs.front().path.c_str());
            sync_settings_from_hardware(settings, devs.front().path);
        }
    }

    emeet_update(ctx, settings);
    return ctx;
}

void emeet_destroy(void *data)
{
    auto *ctx = static_cast<EmeetFilterContext *>(data);
    if (ctx->out_init)
        obs_source_frame_free(&ctx->out_frame);
    delete ctx;
}

void emeet_get_defaults(obs_data_t *settings)
{
    obs_data_set_default_bool(settings, "apply_hw", true);
    obs_data_set_default_int(settings, "hw_zoom", 100);
    obs_data_set_default_bool(settings, "hw_focus_auto", true);
    obs_data_set_default_int(settings, "hw_focus_abs", 508);
    obs_data_set_default_int(settings, "hw_exp_auto", 0);
    obs_data_set_default_int(settings, "hw_exp_abs", 312);
    obs_data_set_default_bool(settings, "hw_wb_auto", true);
    obs_data_set_default_int(settings, "hw_wb_temp", 6500);
    obs_data_set_default_int(settings, "hw_brightness", 128);
    obs_data_set_default_int(settings, "hw_contrast", 128);
    obs_data_set_default_int(settings, "hw_saturation", 128);
    obs_data_set_default_int(settings, "hw_hue", 128);
    obs_data_set_default_int(settings, "hw_sharpness", 128);
    obs_data_set_default_int(settings, "hw_powerline", 1);

    obs_data_set_default_bool(settings, "face_tracking", false);
    obs_data_set_default_double(settings, "track_zoom", 1.6);
    obs_data_set_default_double(settings, "track_smooth", 0.09);
    obs_data_set_default_int(settings, "hdr_mode", 0);
    obs_data_set_default_bool(settings, "lut_enabled", false);
    std::string def_lut_dir;
    if (const char *home = std::getenv("HOME")) {
        std::filesystem::path p1 = std::filesystem::path(home) / ".config" / "LUTs";
        std::filesystem::path p2 = std::filesystem::path(home) / ".config" / "obs-studio" / "LUTs";
        if (std::filesystem::is_directory(p1))
            def_lut_dir = p1.string();
        else if (std::filesystem::is_directory(p2))
            def_lut_dir = p2.string();
    }
    obs_data_set_default_string(settings, "lut_dir", def_lut_dir.c_str());
    obs_data_set_default_string(settings, "lut_file", "");
    obs_data_set_default_string(settings, "lut_path", "");
    obs_data_set_default_bool(settings, "flip_h", false);
    obs_data_set_default_bool(settings, "flip_v", false);
}

std::vector<std::filesystem::path> scan_lut_dir(const std::string &dir_str)
{
    std::vector<std::filesystem::path> out;
    if (dir_str.empty())
        return out;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir_str, ec))
        return out;
    for (const auto &entry : std::filesystem::directory_iterator(dir_str, ec)) {
        if (!entry.is_regular_file(ec))
            continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".png" || ext == ".cube")
            out.push_back(entry.path());
    }
    std::sort(out.begin(), out.end());
    return out;
}

void populate_lut_list(obs_property_t *list, const std::string &dir_str)
{
    obs_property_list_clear(list);
    obs_property_list_add_string(list, "(None)", "");
    for (const auto &p : scan_lut_dir(dir_str)) {
        obs_property_list_add_string(list, p.stem().string().c_str(), p.string().c_str());
    }
}

bool on_lut_dir_changed(obs_properties_t *props, obs_property_t *, obs_data_t *settings)
{
    obs_property_t *list = obs_properties_get(props, "lut_file");
    if (!list)
        return false;
    const char *dir_c = obs_data_get_string(settings, "lut_dir");
    populate_lut_list(list, dir_c ? dir_c : "");
    return true;
}

bool step_lut(void *data, int delta)
{
    auto *ctx = static_cast<EmeetFilterContext *>(data);
    obs_data_t *settings = obs_source_get_settings(ctx->source);
    if (!settings)
        return false;

    const char *dir_c = obs_data_get_string(settings, "lut_dir");
    auto files = scan_lut_dir(dir_c ? dir_c : "");
    if (files.empty()) {
        obs_data_release(settings);
        return false;
    }

    const char *cur_c = obs_data_get_string(settings, "lut_file");
    std::string cur = cur_c ? cur_c : "";
    int idx = -1;
    for (int i = 0; i < static_cast<int>(files.size()); ++i) {
        if (files[i].string() == cur) {
            idx = i;
            break;
        }
    }
    int n = static_cast<int>(files.size());
    int next_idx = (idx < 0) ? (delta > 0 ? 0 : n - 1) : ((idx + delta + n) % n);

    obs_data_set_string(settings, "lut_file", files[next_idx].string().c_str());
    obs_data_set_bool(settings, "lut_enabled", true);
    emeet_update(ctx, settings);
    obs_data_release(settings);
    return true;
}

bool on_lut_prev_clicked(obs_properties_t *, obs_property_t *, void *data)
{
    return step_lut(data, -1);
}

bool on_lut_next_clicked(obs_properties_t *, obs_property_t *, void *data)
{
    return step_lut(data, 1);
}

bool on_reset_hw_clicked(obs_properties_t *, obs_property_t *, void *data)
{
    auto *ctx = static_cast<EmeetFilterContext *>(data);
    obs_data_t *settings = obs_source_get_settings(ctx->source);
    if (!settings)
        return false;

    obs_data_set_int(settings, "hw_zoom", 100);
    obs_data_set_bool(settings, "hw_focus_auto", true);
    obs_data_set_int(settings, "hw_focus_abs", 508);
    obs_data_set_int(settings, "hw_exp_auto", 0);
    obs_data_set_int(settings, "hw_exp_abs", 312);
    obs_data_set_bool(settings, "hw_wb_auto", true);
    obs_data_set_int(settings, "hw_wb_temp", 6500);
    obs_data_set_int(settings, "hw_brightness", 128);
    obs_data_set_int(settings, "hw_contrast", 128);
    obs_data_set_int(settings, "hw_saturation", 128);
    obs_data_set_int(settings, "hw_hue", 128);
    obs_data_set_int(settings, "hw_sharpness", 128);
    obs_data_set_int(settings, "hw_powerline", 1);

    emeet_update(ctx, settings);
    obs_data_release(settings);
    return true;
}

obs_properties_t *emeet_get_properties(void *data)
{
    auto *ctx = static_cast<EmeetFilterContext *>(data);
    obs_properties_t *props = obs_properties_create();

    // --- Hardware V4L2 Group ---
    obs_properties_t *hw_group = obs_properties_create();
    obs_property_t *dev_list = obs_properties_add_list(
        hw_group, "device_path", "Camera V4L2 Device",
        OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);

    for (const auto &d : emeet::list_v4l2_devices()) {
        std::string label = d.name + (d.is_emeet ? " [EMEET]" : "") + " (" + d.path + ")";
        obs_property_list_add_string(dev_list, label.c_str(), d.path.c_str());
    }

    obs_properties_add_bool(hw_group, "apply_hw", "Control Hardware Camera via V4L2");
    obs_properties_add_int_slider(hw_group, "hw_zoom", "Hardware Zoom (100=73° FOV, 400=40°)", 100, 400, 1);
    obs_properties_add_bool(hw_group, "hw_focus_auto", "Continuous Autofocus (PDAF+ToF)");
    obs_properties_add_int_slider(hw_group, "hw_focus_abs", "Manual Focus", 298, 719, 1);

    obs_property_t *exp_list = obs_properties_add_list(
        hw_group, "hw_exp_auto", "Exposure Mode",
        OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    obs_property_list_add_int(exp_list, "Auto (Default)", 0);
    obs_property_list_add_int(exp_list, "Manual Mode", 1);
    obs_property_list_add_int(exp_list, "Shutter Priority", 2);
    obs_properties_add_int_slider(hw_group, "hw_exp_abs", "Manual Exposure Time", 2, 10000, 10);

    obs_properties_add_bool(hw_group, "hw_wb_auto", "Auto White Balance");
    obs_properties_add_int_slider(hw_group, "hw_wb_temp", "Color Temperature (K)", 2300, 7500, 50);

    obs_properties_add_int_slider(hw_group, "hw_brightness", "Brightness", 0, 255, 1);
    obs_properties_add_int_slider(hw_group, "hw_contrast", "Contrast", 0, 255, 1);
    obs_properties_add_int_slider(hw_group, "hw_saturation", "Saturation", 0, 255, 1);
    obs_properties_add_int_slider(hw_group, "hw_hue", "Hue", 0, 255, 1);
    obs_properties_add_int_slider(hw_group, "hw_sharpness", "Sharpness", 0, 255, 1);

    obs_property_t *pl_list = obs_properties_add_list(
        hw_group, "hw_powerline", "Anti-Flicker",
        OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    obs_property_list_add_int(pl_list, "Disabled", 0);
    obs_property_list_add_int(pl_list, "50 Hz", 1);
    obs_property_list_add_int(pl_list, "60 Hz", 2);

    obs_properties_add_button(hw_group, "reset_hw", "Reset Hardware Controls", on_reset_hw_clicked);
    obs_properties_add_group(props, "hw_group", "Hardware Controls (V4L2 / UVC)", OBS_GROUP_NORMAL, hw_group);

    // --- Software Filter Group ---
    obs_properties_t *sw_group = obs_properties_create();
    obs_properties_add_bool(sw_group, "face_tracking", "Enable OpenCV Face Tracking (Auto-Framing)");
    obs_properties_add_float_slider(sw_group, "track_zoom", "Tracking Zoom", 1.1, 3.0, 0.1);
    obs_properties_add_float_slider(sw_group, "track_smooth", "Tracking Smoothing", 0.02, 0.25, 0.01);

    obs_property_t *hdr_list = obs_properties_add_list(
        sw_group, "hdr_mode", "Software HDR (Shadow Lift)",
        OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    obs_property_list_add_int(hdr_list, "Off", 0);
    obs_property_list_add_int(hdr_list, "Balanced", 1);
    obs_property_list_add_int(hdr_list, "Strong", 2);

    obs_properties_add_bool(sw_group, "lut_enabled", "Enable 3D LUT (On / Off)");
    obs_property_t *lut_dir_prop = obs_properties_add_path(
        sw_group, "lut_dir", "LUT Folder",
        OBS_PATH_DIRECTORY, nullptr, nullptr);
    obs_property_set_modified_callback(lut_dir_prop, on_lut_dir_changed);

    obs_property_t *lut_list = obs_properties_add_list(
        sw_group, "lut_file", "Select LUT",
        OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
    {
        obs_data_t *cur_settings = obs_source_get_settings(ctx->source);
        const char *dir_c = cur_settings ? obs_data_get_string(cur_settings, "lut_dir") : "";
        populate_lut_list(lut_list, dir_c ? dir_c : "");
        if (cur_settings)
            obs_data_release(cur_settings);
    }
    obs_properties_add_button(sw_group, "lut_prev", "◀ Previous LUT", on_lut_prev_clicked);
    obs_properties_add_button(sw_group, "lut_next", "Next LUT ▶", on_lut_next_clicked);

    obs_properties_add_bool(sw_group, "flip_h", "Flip Horizontal (Mirror)");
    obs_properties_add_bool(sw_group, "flip_v", "Flip Vertical");

    obs_properties_add_group(props, "sw_group", "Software Effects (No Virtual Cam)", OBS_GROUP_NORMAL, sw_group);

    (void)ctx;
    return props;
}

struct obs_source_frame *emeet_filter_video(void *data, struct obs_source_frame *frame)
{
    if (!frame)
        return nullptr;

    auto *ctx = static_cast<EmeetFilterContext *>(data);
    std::lock_guard<std::mutex> lock(ctx->mtx);

    const bool need_lut = ctx->lut_enabled && ctx->lut.is_loaded();
    const bool need_hdr = ctx->hdr_mode != emeet::HdrMode::Off;
    const bool need_sw = ctx->face_tracking || need_hdr || need_lut || ctx->flip_h || ctx->flip_v;

    // 0% CPU bypass when only hardware V4L2 controls are used
    if (!need_sw)
        return frame;

    cv::Mat &bgr = ctx->work_bgr;
    if (!frame_to_bgr(frame, bgr, ctx->work_yuv))
        return frame;

    if (ctx->flip_h || ctx->flip_v) {
        int flip_code = (ctx->flip_h && ctx->flip_v) ? -1 : (ctx->flip_h ? 1 : 0);
        cv::flip(bgr, bgr, flip_code);
    }

    if (ctx->face_tracking)
        ctx->tracker.process(bgr, ctx->track_zoom, ctx->track_smooth);

    if (need_hdr)
        emeet::SoftwareHdr::apply(bgr, ctx->hdr_mode);

    if (need_lut)
        ctx->lut.apply(bgr);

    ensure_out_frame(ctx, static_cast<uint32_t>(bgr.cols), static_cast<uint32_t>(bgr.rows));
    cv::Mat out_bgra(bgr.rows, bgr.cols, CV_8UC4, ctx->out_frame.data[0], ctx->out_frame.linesize[0]);
    cv::cvtColor(bgr, out_bgra, cv::COLOR_BGR2BGRA);

    ctx->out_frame.timestamp = frame->timestamp;
    return &ctx->out_frame;
}

struct obs_source_info emeet_filter_info = {
    .id = "emeet_camcontrol_filter",
    .type = OBS_SOURCE_TYPE_FILTER,
    .output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_ASYNC,
    .get_name = emeet_get_name,
    .create = emeet_create,
    .destroy = emeet_destroy,
    .get_defaults = emeet_get_defaults,
    .get_properties = emeet_get_properties,
    .update = emeet_update,
    .filter_video = emeet_filter_video,
};

} // namespace

bool obs_module_load(void)
{
    // Prevent OpenCV/TBB from spawning 32+ worker threads & arenas inside OBS's graphics thread
    cv::setNumThreads(1);
    obs_register_source(&emeet_filter_info);
    return true;
}
