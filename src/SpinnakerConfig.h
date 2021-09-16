
#ifndef __CAMERA_SPINNAKER__SPINNAKERCONFIG_H__
#define __CAMERA_SPINNAKER__SPINNAKERCONFIG_H__

#include <string>

namespace camera_spinnaker
{
  struct SpinnakerConfig
  {
    double acquisition_frame_rate;
    bool acquisition_frame_rate_enable;
    std::string exposure_mode;
    std::string exposure_auto;
    double exposure_time;
    double auto_exposure_time_upper_limit;
    std::string gain_selector;
    std::string auto_gain;
    double gain;
    double brightness;
    bool sharpening_enable;
    bool auto_sharpness;
    double sharpness;
    double sharpening_threshold;
    bool saturation_enable;
    double saturation;
    bool gamma_enable;
    double gamma;
    std::string auto_white_balance;
    double white_balance_blue_ratio;
    double white_balance_red_ratio;
    int image_format_roi_width;
    int image_format_roi_height;
    int image_format_x_offset;
    int image_format_y_offset;
    int image_format_x_binning;
    int image_format_y_binning;
    int image_format_x_decimation;
    int image_format_y_decimation;
    bool image_format_x_reverse;
    bool image_format_y_reverse;
    std::string image_format_color_coding;
    std::string enable_trigger;
    std::string trigger_selector;
    std::string trigger_activation_mode;
    std::string trigger_source;
    std::string trigger_overlap_mode;
    std::string line_source;
    std::string line_selector;
    std::string line_mode;
    int auto_exposure_roi_offset_x;
    int auto_exposure_roi_offset_y;
    int auto_exposure_roi_width;
    int auto_exposure_roi_height;
    std::string auto_exposure_lighting_mode;
    double time_offset;
    bool state;
    std::string name;

    /** Default values **/
    SpinnakerConfig()
    {
        acquisition_frame_rate = 10.0;
        acquisition_frame_rate_enable = 0;
        exposure_mode = "Timed";
        exposure_auto = "Once";
        exposure_time = 100.0;
        auto_exposure_time_upper_limit = 30000.0;
        gain_selector = "All";
        auto_gain = "Continuous";
        gain = 0.0;
        brightness = 1.7;
        sharpening_enable = 0;
        auto_sharpness = 1;
        sharpness = 1024.0;
        sharpening_threshold = 0.1;
        saturation_enable = 0;
        saturation = 100.0;
        gamma_enable = 1;
        gamma = 1.0;
        auto_white_balance = "Continuous";
        white_balance_blue_ratio = 800.0;
        white_balance_red_ratio = 550.0;
        image_format_roi_width = 0;
        image_format_roi_height = 0;
        image_format_x_offset = 0;
        image_format_y_offset = 0;
        image_format_x_binning = 1;
        image_format_y_binning = 1;
        image_format_x_decimation = 1;
        image_format_y_decimation = 1;
        image_format_x_reverse = 0;
        image_format_y_reverse = 0;
        image_format_color_coding = "BGR8";
        enable_trigger = "Off";
        trigger_selector = "FrameStart";
        trigger_activation_mode = "FallingEdge";
        trigger_source = "Line0";
        trigger_overlap_mode = "ReadOut";
        line_source = "Off";
        line_selector = "Line0";
        line_mode = "Input";
        auto_exposure_roi_offset_x = 0;
        auto_exposure_roi_offset_y = 0;
        auto_exposure_roi_width = 0;
        auto_exposure_roi_height = 0;
        auto_exposure_lighting_mode = "Normal";
        time_offset = 0.0;
    }

    };
}
#endif