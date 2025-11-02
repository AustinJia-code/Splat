/**
 * @file OV9715.cpp
 * @brief Class for stereo camera
 * @details Run info:
 *            1. sudo apt install libopencv-dev
 *            2. Run calibrate.cpp or upload a calibration yml
 *            3. g++ OV9715.cpp -o OV9715 `pkg-config --cflags --libs opencv4`
 *            4. ./OV9715
 * @cite Useful information from:
 * https://www.cs.cmu.edu/~16385/s17/Slides/13.1_Stereo_Rectification.pdf
 * https://www.cs.cmu.edu/~16385/s17/Slides/13.2_Stereo_Matching.pdf
 * https://learnopencv.com/camera-calibration-using-opencv/
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cassert>
#include <fstream>

class OV9715
{
public:
    /**
     * Constructor with calibration file and port
     */
    OV9715 (const std::string& calib_file = "../src/config/stereo_calib.yml", 
            int port = -1)
    {
        loadCalibration (calib_file);
        if (port != -1)
            stereo_cam = cv::VideoCapture (port);
    }

    /**
     * Load image for debugging
     */
    void load (const std::string& in_path,
               const std::string& id = "", 
               const std::string& out_path = "../data/out-1-cam/")
    {
        // Capture
        this->raw_frame = cv::imread (in_path);
        this->img_size = cv::Size (raw_frame.cols / 2, raw_frame.rows);
        this->split ();

        // Save
        this->save (out_path, id, this->left_raw, this->right_raw);
    }

    /**
     * Camera raw combined -> raw split
     */
    void capture (const std::string& id = "", 
                  const std::string& path = "../data/out-1-cam/")
    {
        // Capture
        this->stereo_cam >> this->raw_frame;
        this->img_size = cv::Size (raw_frame.cols / 2, raw_frame.rows);
        this->split ();

        // Save
        this->save (path, id, this->left_raw, this->right_raw);
    }

    /**
     * Raw split -> undistorted, rectified split
     */
    void undistort (const std::string& id = "", 
                    const std::string& path = "../data/out-2-undistort/")
    {
        assert (!left_raw.empty () && !right_raw.empty ());

        // Undistort and rectify
        cv::Mat map1_left, map2_left, map1_right, map2_right;
        cv::initUndistortRectifyMap
        (
            K1, D1, R1, P1,
            img_size, CV_32FC1,
            map1_left, map2_left
        );

        cv::initUndistortRectifyMap
        (
            K2, D2, R2, P2,
            img_size, CV_32FC1,
            map1_right, map2_right
        );

        cv::remap
        (
            this->left_raw, this->left_rect, 
            map1_left, map2_left,
            cv::INTER_LINEAR
        );

        cv::remap
        (
            this->right_raw, this->right_rect,
            map1_right, map2_right,
            cv::INTER_LINEAR
        );

        // Save
        this->save (path, id, this->left_rect, this->right_rect);
    }

    /**
     * Rectified split -> depth
     */
    void disparity (const std::string& id = "", 
                    const std::string& path = "../data/out-3-disparity/")
    {
        assert (!this->left_rect.empty () && !this->right_rect.empty ());
        // Convert to grayscale
        cv::Mat left_gray, right_gray;
        cv::cvtColor (this->left_rect, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor (this->right_rect, right_gray, cv::COLOR_BGR2GRAY);

        // Compute disparity
        int numDisparities = 16 * 6;
        int blockSize = 11;
        cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create
        (
            0, numDisparities, blockSize,
            8 * 3 * blockSize * blockSize,
            32 * 3 * blockSize * blockSize,
            1, 63, 10, 100, 32,
            cv::StereoSGBM::MODE_SGBM_3WAY
        );

        cv::Mat disparity8U;
        stereo->compute (left_gray, right_gray, this->disparity_map);

        // Visualize and save
        if (!id.empty ())
        {
            this->disparity_map.convertTo (disparity8U, CV_8U, 255.0 / 
                                          (numDisparities * 16));
            cv::Mat disparity_color;
            cv::applyColorMap (disparity8U, disparity_color, cv::COLORMAP_JET);
            this->save (path, id, disparity_color);
        }
    }

    /**
     * Disparity -> depth
     */
    void depth (const std::string& id = "", 
                const std::string& path = "../data/out-4-depth/",
                const std::array<int, 2>& clip_range_mm = {0, 5000})
    {
        assert (!this->disparity_map.empty ());
        cv::Mat disparity_float;
        this->disparity_map.convertTo (disparity_float, CV_32F, 1.0 / 16.0);
        this->depth_map = cv::Mat (disparity_float.size (), CV_32F);
        float fx = P1.at<double> (0, 0);
        float B = std::abs (P2.at<double> (0, 3)) / fx;

        // Calculate disparity for each point
        for (int y = 0; y < disparity_float.rows; y++)
            for (int x = 0; x < disparity_float.cols; x++)
            {
                float disparity = disparity_float.at<float> (y, x);
                float depth = 0.0f;
                if (disparity > 0.1f)
                    depth = fx * B / disparity;
                
                if (depth < clip_range_mm[0]) depth = clip_range_mm[0];
                if (depth > clip_range_mm[1]) depth = clip_range_mm[1];
                
                this->depth_map.at<float> (y, x) = depth;
            }
        
        // Visualize and save
        if (!id.empty())
        {
            // Scale depth
            cv::Mat depth_vis;
            float min_val = static_cast<float> (clip_range_mm[0]);
            float max_val = static_cast<float> (clip_range_mm[1]);

            // Map clipped depth range to 0–255
            depth_map.convertTo (depth_vis, CV_8U, 255.0 / (max_val - min_val), 
                                 -min_val * 255.0 / (max_val - min_val));

            cv::Mat depth_color;
            cv::applyColorMap(depth_vis, depth_color, cv::COLORMAP_JET);

            // Save
            this->save (path, id, depth_color);
        }
    }

    /**
     * Depth -> point cloud
     */
    void points (const std::string& id = "", 
                 const std::string& path = "../data/out-5-points/")
    {
        assert(!this->depth_map.empty () && !this->left_rect.empty ());

        // Get camera intrinsics
        float fx = P1.at<double> (0, 0);
        float fy = P1.at<double> (1, 1);
        float cx = P1.at<double> (0, 2);
        float cy = P1.at<double> (1, 2);

        // Prepare output
        std::vector<cv::Vec3f> points_3D;
        std::vector<cv::Vec3b> colors;

        // Convert depth map to 3D points
        for (int y = 0; y < depth_map.rows; y++)
            for (int x = 0; x < depth_map.cols; x++)
            {
                float depth = depth_map.at<float> (y, x);
                
                // Skip invalid depths
                if (depth <= 0.0f)
                    continue;

                // Backproject to 3D using pinhole camera model
                float X = (x - cx) * depth / fx;
                float Y = (y - cy) * depth / fy;
                float Z = depth;

                points_3D.push_back (cv::Vec3f (X, Y, Z));
                colors.push_back (left_rect.at<cv::Vec3b> (y, x));
            }

        // Write to .ply
        if (!id.empty ())
        {
            std::string filename = path + id + ".ply";
            this->save_ply (filename, points_3D, colors);
        }
    }

    /**
     * Full capture -> point cloud pipeline
     */
    void process (const std::string& id)
    {
        this->capture (id);
        this->undistort (id);
        this->disparity (id);
        this->depth (id);
        this->points (id);
    }

private:
    cv::VideoCapture stereo_cam;

    // Stereo calibration matrices
    cv::Mat K1, D1, K2, D2;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size img_size;

    // Images
    cv::Mat raw_frame;
    cv::Mat left_raw, right_raw;
    cv::Mat left_rect, right_rect;
    cv::Mat disparity_map;
    cv::Mat depth_map;

    /**
     * Load calibration data from yml
     */
    void loadCalibration (const std::string& calib_file)
    {
        cv::FileStorage fs (calib_file, cv::FileStorage::READ);
        if (!fs.isOpened ())
        {
            std::cerr << "Failed to open calibration file: " << calib_file 
                      << std::endl;
            std::exit (EXIT_FAILURE);
        }

        fs["K1"] >> K1;
        fs["D1"] >> D1;
        fs["K2"] >> K2;
        fs["D2"] >> D2;
        fs["R1"] >> R1;
        fs["R2"] >> R2;
        fs["P1"] >> P1;
        fs["P2"] >> P2;
        fs["Q"] >> Q;

        fs.release ();
    }   

    /**
     * Image saver helper
     */
    void save (const std::string& path, const std::string& id,
               const cv::Mat& left, const cv::Mat& right = cv::Mat ())
    {
        if (id.empty ())
            return;

        std::string cur_path = path + id;

        if (right.empty ())
            cv::imwrite (cur_path + ".jpg", left);
        else
        {
            cv::imwrite (cur_path + "_left.jpg", left);
            cv::imwrite (cur_path + "_right.jpg", right);
        }
        
        std::cout << "Saved " << id << " to " << cur_path 
                  << "*.jpg" << std::endl;
    }

    /**
     * Raw cam split helper
     */
    void split ()
    {
        int rows = this->raw_frame.rows;
        int cols = this->raw_frame.cols;
        this->left_raw = raw_frame (cv::Range (0, rows), 
                                    cv::Range (0, cols / 2)).clone ();
        this->right_raw = raw_frame (cv::Range (0, rows),
                                     cv::Range (cols / 2, cols)).clone ();
    }

    /**
     * .ply save helper
     */
    void save_ply (const std::string& filename, 
                   const std::vector<cv::Vec3f>& points,
                   const std::vector<cv::Vec3b>& colors)
    {
        std::ofstream ofs (filename);

        // Write header
        ofs << "ply\nformat ascii 1.0\n";
        ofs << "element vertex " << points.size () << "\n";
        ofs << "property float x\nproperty float y\nproperty float z\n";
        ofs << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
        ofs << "end_header\n";

        // Write data
        for (size_t i = 0; i < points.size (); ++i)
        {
            const cv::Vec3f& point = points[i];
            const cv::Vec3b& color = colors[i];

            ofs << point[0] << " " << point[1] << " " << point[2] << " "
                << (int) color[2] << " " << (int) color[1] << " " << (int) color[0]
                << "\n";
        }

        ofs.close ();
        std::cout << "Saved to " << filename << std::endl;
    }
};