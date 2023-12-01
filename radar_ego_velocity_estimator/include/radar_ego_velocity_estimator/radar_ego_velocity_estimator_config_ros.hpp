#pragma once


namespace reve
{
class RadarEgoVelocityEstimatorConfig
{
    public:
        double min_dist;                          
        double max_dist;                           
        double min_db;
        double elevation_thresh_deg;
        double azimuth_thresh_deg;
        double filter_min_z;
        double filter_max_z;
        double doppler_velocity_correction_factor;
        double thresh_zero_velocity;
        double allowed_outlier_percentage;
        double sigma_zero_velocity_x;
        double sigma_zero_velocity_y;
        double sigma_zero_velocity_z;
        double sigma_offset_radar_x;
        double sigma_offset_radar_y;
        double sigma_offset_radar_z;
        double max_sigma_x;
        double max_sigma_y;
        double max_sigma_z;
        double max_r_cond;
        bool   use_cholesky_instead_of_bdcsvd;
        bool   use_ransac;
        double outlier_prob;
        double success_prob;
        int    N_ransac_points;
        double inlier_thresh;
        bool   use_odr;
        double sigma_v_d;
        double min_speed_odr;
        double model_noise_offset_deg;
        double model_noise_scale_deg;
};
}  // namespace reve