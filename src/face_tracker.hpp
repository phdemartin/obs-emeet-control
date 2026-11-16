#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace emeet {

class FaceTracker {
public:
    FaceTracker()
    {
        const std::vector<std::string> front_paths = {
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt2.xml",
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml",
        };
        for (const auto &p : front_paths) {
            if (front_cascade_.load(p))
                break;
        }
        const std::vector<std::string> prof_paths = {
            "/usr/share/opencv4/haarcascades/haarcascade_profileface.xml",
            "/usr/share/opencv/haarcascades/haarcascade_profileface.xml",
        };
        for (const auto &p : prof_paths) {
            if (profile_cascade_.load(p))
                break;
        }
    }

    bool is_ready() const { return !front_cascade_.empty(); }

    void reset()
    {
        has_pos_ = false;
        has_target_ = false;
        face_h_ratio_ = 0.25f;
        frame_idx_ = 0;
    }

    void process(cv::Mat &bgr, double zoom_factor, double smoothing)
    {
        if (bgr.empty() || !is_ready())
            return;

        const int w = bgr.cols;
        const int h = bgr.rows;
        if (!has_pos_) {
            cx_ = 0.5f;
            cy_ = 0.45f;
            has_pos_ = true;
        }

        if (frame_idx_ % 2 == 0) {
            detect(bgr);
        }
        frame_idx_++;

        const float smooth = static_cast<float>(std::clamp(smoothing, 0.02, 0.35));
        constexpr float deadzone = 0.04f;
        constexpr float max_step = 0.015f;

        if (has_target_) {
            face_h_ratio_ = face_h_ratio_ * 0.85f + target_fh_ * 0.15f;
            float dx = target_x_ - cx_;
            float dy = target_y_ - cy_;
            if (std::abs(dx) > deadzone)
                cx_ += std::clamp(dx * smooth, -max_step, max_step);
            if (std::abs(dy) > deadzone)
                cy_ += std::clamp(dy * smooth, -max_step, max_step);
        }

        const float z = static_cast<float>(std::clamp(zoom_factor, 1.05, 3.5));
        int min_crop_h = static_cast<int>(h * std::min(1.0f, face_h_ratio_ * 2.8f));
        int desired_crop_h = static_cast<int>(h / z);
        int crop_h = std::max(desired_crop_h, min_crop_h) & ~1;
        crop_h = std::clamp(crop_h, 120, h);

        int crop_w = static_cast<int>(crop_h * (static_cast<float>(w) / h)) & ~1;
        crop_w = std::clamp(crop_w, 120, w);
        crop_h = std::min(h, static_cast<int>(crop_w * (static_cast<float>(h) / w)) & ~1);

        int x = static_cast<int>(std::lround(cx_ * w - crop_w * 0.5f));
        int y = static_cast<int>(std::lround(cy_ * h - crop_h * 0.45f));
        x = std::clamp(x, 0, w - crop_w);
        y = std::clamp(y, 0, h - crop_h);

        if (crop_w < w || crop_h < h) {
            cv::Mat cropped = bgr(cv::Rect(x, y, crop_w, crop_h));
            cv::Mat resized;
            cv::resize(cropped, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
            bgr = std::move(resized);
        }
    }

private:
    void detect(const cv::Mat &bgr)
    {
        const int target_w = 320;
        const float scale = static_cast<float>(target_w) / bgr.cols;
        const int target_h = std::max(1, static_cast<int>(bgr.rows * scale));

        cv::Mat small, gray;
        cv::resize(bgr, small, cv::Size(target_w, target_h), 0, 0, cv::INTER_AREA);
        cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);

        const int min_sz = std::max(24, static_cast<int>(target_h * 0.10f));
        std::vector<cv::Rect> faces;
        front_cascade_.detectMultiScale(gray, faces, 1.12, 5, 0, cv::Size(min_sz, min_sz));
        if (faces.empty() && !profile_cascade_.empty()) {
            profile_cascade_.detectMultiScale(gray, faces, 1.15, 5, 0, cv::Size(min_sz, min_sz));
        }
        if (faces.empty())
            return;

        auto best = *std::max_element(faces.begin(), faces.end(), [](const cv::Rect &a, const cv::Rect &b) {
            return a.area() < b.area();
        });

        target_x_ = (best.x + best.width * 0.5f) / static_cast<float>(gray.cols);
        target_y_ = (best.y + best.height * 0.52f) / static_cast<float>(gray.rows);
        target_fh_ = best.height / static_cast<float>(gray.rows);
        has_target_ = true;
    }

    cv::CascadeClassifier front_cascade_;
    cv::CascadeClassifier profile_cascade_;
    bool has_pos_ = false;
    bool has_target_ = false;
    float cx_ = 0.5f;
    float cy_ = 0.45f;
    float target_x_ = 0.5f;
    float target_y_ = 0.45f;
    float target_fh_ = 0.25f;
    float face_h_ratio_ = 0.25f;
    int frame_idx_ = 0;
};

} // namespace emeet
