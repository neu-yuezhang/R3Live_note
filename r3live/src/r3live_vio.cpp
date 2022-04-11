#include "r3live.hpp"
// #include "photometric_error.hpp"
#include "tools_mem_used.h"
#include "tools_logger.hpp"
#include <Eigen/Core>
using namespace std;

Common_tools::Cost_time_logger g_cost_time_logger;
std::shared_ptr<Common_tools::ThreadPool> m_thread_pool_ptr;
double g_vio_frame_cost_time = 0;
double g_lio_frame_cost_time = 0;
int g_flag_if_first_rec_img = 1;

#define USING_CERES 0
void dump_lio_state_to_log(FILE *fp)
{
        if (fp != nullptr && g_camera_lidar_queue.m_if_dump_log)
        {
                Eigen::Vector3d rot_angle = Sophus::SO3d(Eigen::Quaterniond(g_lio_state.rot_end)).log();
                Eigen::Vector3d rot_ext_i2c_angle = Sophus::SO3d(Eigen::Quaterniond(g_lio_state.rot_ext_i2c)).log();
                fprintf(fp, "%lf ", g_lio_state.last_update_time - g_camera_lidar_queue.m_first_imu_time); // Time   [0]
                fprintf(fp, "%lf %lf %lf ", rot_angle(0), rot_angle(1), rot_angle(2));                     // Angle  [1-3]
                fprintf(fp, "%lf %lf %lf ", g_lio_state.pos_end(0), g_lio_state.pos_end(1),
                        g_lio_state.pos_end(2));            // Pos    [4-6]
                fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega  [7-9]
                fprintf(fp, "%lf %lf %lf ", g_lio_state.vel_end(0), g_lio_state.vel_end(1),
                        g_lio_state.vel_end(2));            // Vel    [10-12]
                fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // Acc    [13-15]
                fprintf(fp, "%lf %lf %lf ", g_lio_state.bias_g(0), g_lio_state.bias_g(1),
                        g_lio_state.bias_g(2)); // Bias_g [16-18]
                fprintf(fp, "%lf %lf %lf ", g_lio_state.bias_a(0), g_lio_state.bias_a(1),
                        g_lio_state.bias_a(2)); // Bias_a [19-21]
                fprintf(fp, "%lf %lf %lf ", g_lio_state.gravity(0), g_lio_state.gravity(1),
                        g_lio_state.gravity(2)); // Gravity[22-24]
                fprintf(fp, "%lf %lf %lf ", rot_ext_i2c_angle(0), rot_ext_i2c_angle(1),
                        rot_ext_i2c_angle(2)); // Rot_ext_i2c[25-27]
                fprintf(fp, "%lf %lf %lf ", g_lio_state.pos_ext_i2c(0), g_lio_state.pos_ext_i2c(1),
                        g_lio_state.pos_ext_i2c(2)); // pos_ext_i2c [28-30]
                fprintf(fp, "%lf %lf %lf %lf ", g_lio_state.cam_intrinsic(0), g_lio_state.cam_intrinsic(1), g_lio_state.cam_intrinsic(2),
                        g_lio_state.cam_intrinsic(3));       // Camera Intrinsic [31-34]
                fprintf(fp, "%lf ", g_lio_state.td_ext_i2c); // Camera Intrinsic [35]
                // cout <<  g_lio_state.cov.diagonal().transpose() << endl;
                // cout <<  g_lio_state.cov.block(0,0,3,3) << endl;
                for (int idx = 0; idx < DIM_OF_STATES; idx++) // Cov    [36-64]
                {
                        fprintf(fp, "%.9f ", sqrt(g_lio_state.cov(idx, idx)));
                }
                fprintf(fp, "%lf %lf ", g_lio_frame_cost_time, g_vio_frame_cost_time); // costime [65-66]
                fprintf(fp, "\r\n");
                fflush(fp);
        }
}

double g_last_stamped_mem_mb = 0;
std::string append_space_to_bits(std::string &in_str, int bits)
{
        while (in_str.length() < bits)
        {
                in_str.append(" ");
        }
        return in_str;
}
void R3LIVE::print_dash_board()
{
        /*int mem_used_mb = (int)(Common_tools::get_RSS_Mb());
        // clang-format off
    if( (mem_used_mb - g_last_stamped_mem_mb < 1024 ) && g_last_stamped_mem_mb != 0 )
    {
        cout  << ANSI_DELETE_CURRENT_LINE << ANSI_DELETE_LAST_LINE ;
    }
    else
    {
        cout << "\r\n" << endl;
        cout << ANSI_COLOR_WHITE_BOLD << "======================= R3LIVE Dashboard ======================" << ANSI_COLOR_RESET << endl;
        g_last_stamped_mem_mb = mem_used_mb ;
    }
    std::string out_str_line_1, out_str_line_2;
    out_str_line_1 = std::string(        "| System-time | LiDAR-frame | Camera-frame |  Pts in maps | Memory used (Mb) |") ;
    //                                    1             16            30             45             60
        // clang-format on
        out_str_line_2.reserve(1e3);
        out_str_line_2.append("|   ").append(Common_tools::get_current_time_str());
        append_space_to_bits(out_str_line_2, 14);
        out_str_line_2.append("|    ").append(std::to_string(g_LiDAR_frame_index));
        append_space_to_bits(out_str_line_2, 28);
        out_str_line_2.append("|    ").append(std::to_string(g_camera_frame_idx));
        append_space_to_bits(out_str_line_2, 43);
        out_str_line_2.append("| ").append(std::to_string(m_map_rgb_pts.m_rgb_pts_vec.size()));
        append_space_to_bits(out_str_line_2, 58);
        out_str_line_2.append("|    ").append(std::to_string(mem_used_mb));

        out_str_line_2.insert(58, ANSI_COLOR_YELLOW, 7);
        out_str_line_2.insert(43, ANSI_COLOR_BLUE, 7);
        out_str_line_2.insert(28, ANSI_COLOR_GREEN, 7);
        out_str_line_2.insert(14, ANSI_COLOR_RED, 7);
        out_str_line_2.insert(0, ANSI_COLOR_WHITE, 7);

        out_str_line_1.insert(58, ANSI_COLOR_YELLOW_BOLD, 7);
        out_str_line_1.insert(43, ANSI_COLOR_BLUE_BOLD, 7);
        out_str_line_1.insert(28, ANSI_COLOR_GREEN_BOLD, 7);
        out_str_line_1.insert(14, ANSI_COLOR_RED_BOLD, 7);
        out_str_line_1.insert(0, ANSI_COLOR_WHITE_BOLD, 7);

        cout << out_str_line_1 << endl;
        cout << out_str_line_2 << ANSI_COLOR_RESET << "          ";
        ANSI_SCREEN_FLUSH;*/
}

void R3LIVE::set_initial_state_cov(StatesGroup &state)
{
        // Set cov
        scope_color(ANSI_COLOR_RED_BOLD);
        state.cov = state.cov.setIdentity() * INIT_COV;
        // state.cov.block(18, 18, 6 , 6 ) = state.cov.block(18, 18, 6 , 6 ) .setIdentity() * 0.1;
        // state.cov.block(24, 24, 5 , 5 ) = state.cov.block(24, 24, 5 , 5 ).setIdentity() * 0.001;
        state.cov.block(0, 0, 3, 3) = mat_3_3::Identity() * 1e-5;   // R
        state.cov.block(3, 3, 3, 3) = mat_3_3::Identity() * 1e-5;   // T
        state.cov.block(6, 6, 3, 3) = mat_3_3::Identity() * 1e-5;   // vel
        state.cov.block(9, 9, 3, 3) = mat_3_3::Identity() * 1e-3;   // bias_g
        state.cov.block(12, 12, 3, 3) = mat_3_3::Identity() * 1e-1; // bias_a
        state.cov.block(15, 15, 3, 3) = mat_3_3::Identity() * 1e-5; // Gravity
        state.cov(24, 24) = 0.00001;
        state.cov.block(18, 18, 6, 6) = state.cov.block(18, 18, 6, 6).setIdentity() * 1e-3; // Extrinsic between camera and IMU.
        state.cov.block(25, 25, 4, 4) = state.cov.block(25, 25, 4, 4).setIdentity() * 1e-3; // Camera intrinsic.
}

cv::Mat R3LIVE::generate_control_panel_img()
{
        int line_y = 40;
        int padding_x = 10;
        int padding_y = line_y * 0.7;
        cv::Mat res_image = cv::Mat(line_y * 3 + 1 * padding_y, 960, CV_8UC3, cv::Scalar::all(0));
        char temp_char[128];
        sprintf(temp_char, "Click this windows to enable the keyboard controls.");
        cv::putText(res_image, std::string(temp_char), cv::Point(padding_x, line_y * 0 + padding_y), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(0, 255, 255), 2, 8, 0);
        sprintf(temp_char, "Press 'S' or 's' key to save current map");
        cv::putText(res_image, std::string(temp_char), cv::Point(padding_x, line_y * 1 + padding_y), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(255, 255, 255), 2, 8, 0);
        sprintf(temp_char, "Press 'space' key to pause the mapping process");
        cv::putText(res_image, std::string(temp_char), cv::Point(padding_x, line_y * 2 + padding_y), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(255, 255, 255), 2, 8, 0);
        return res_image;
}

void R3LIVE::set_initial_camera_parameter(StatesGroup &state, double *intrinsic_data, double *camera_dist_data, double *imu_camera_ext_R,
                                          double *imu_camera_ext_t, double cam_k_scale)
{
        scope_color(ANSI_COLOR_YELLOW_BOLD);
        // g_cam_K << 863.4241 / cam_k_scale, 0, 625.6808 / cam_k_scale,
        //     0, 863.4171 / cam_k_scale, 518.3392 / cam_k_scale,
        //     0, 0, 1;

        g_cam_K << intrinsic_data[0] / cam_k_scale, intrinsic_data[1], intrinsic_data[2] / cam_k_scale, intrinsic_data[3],
            intrinsic_data[4] / cam_k_scale, intrinsic_data[5] / cam_k_scale, intrinsic_data[6], intrinsic_data[7], intrinsic_data[8];
        g_cam_dist = Eigen::Map<Eigen::Matrix<double, 5, 1>>(camera_dist_data);
        state.rot_ext_i2c = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(imu_camera_ext_R);
        state.pos_ext_i2c = Eigen::Map<Eigen::Matrix<double, 3, 1>>(imu_camera_ext_t);
        // state.pos_ext_i2c.setZero();

        // Lidar to camera parameters.
        m_mutex_lio_process.lock();

        m_inital_rot_ext_i2c = state.rot_ext_i2c;
        m_inital_pos_ext_i2c = state.pos_ext_i2c;
        state.cam_intrinsic(0) = g_cam_K(0, 0);
        state.cam_intrinsic(1) = g_cam_K(1, 1);
        state.cam_intrinsic(2) = g_cam_K(0, 2);
        state.cam_intrinsic(3) = g_cam_K(1, 2);
        set_initial_state_cov(state);
        m_mutex_lio_process.unlock();
}

void R3LIVE::publish_track_img(cv::Mat &img, double frame_cost_time = -1)
{
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        cv::Mat pub_image = img.clone();
        if (frame_cost_time > 0)
        {
                char fps_char[100];
                sprintf(fps_char, "Per-frame cost time: %.2f ms", frame_cost_time);
                // sprintf(fps_char, "%.2f ms", frame_cost_time);

                if (pub_image.cols <= 640)
                {
                        cv::putText(pub_image, std::string(fps_char), cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255), 2, 8,
                                    0); // 640 * 480
                }
                else if (pub_image.cols > 640)
                {
                        cv::putText(pub_image, std::string(fps_char), cv::Point(30, 50), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(255, 255, 255), 2, 8,
                                    0); // 1280 * 1080
                }
        }
        out_msg.image = pub_image; // Your cv::Mat
        pub_track_img.publish(out_msg);
}

void R3LIVE::publish_raw_img(cv::Mat &img)
{
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image = img;                                   // Your cv::Mat
        pub_raw_img.publish(out_msg);
}

int sub_image_typed = 0; // 0: TBD 1: sub_raw, 2: sub_comp
std::mutex mutex_image_callback;

std::deque<sensor_msgs::CompressedImageConstPtr> g_received_compressed_img_msg; //保存图像msg的队列
std::deque<sensor_msgs::ImageConstPtr> g_received_img_msg;
std::shared_ptr<std::thread> g_thr_process_image;

void R3LIVE::service_process_img_buffer()
{
        while (1)
        {
                // cout << "在2" << endl;
                //  To avoid uncompress so much image buffer, reducing the use of memory.
                //避免解压缩如此多的图像缓冲区，减少内存的使用。
                if (m_queue_image_with_pose.size() > 4)
                {
                        while (m_queue_image_with_pose.size() > 4)
                        {
                                ros::spinOnce();
                                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                                std::this_thread::yield();
                        }
                }
                cv::Mat image_get;
                double img_rec_time;
                if (sub_image_typed == 2)
                {
                        while (g_received_compressed_img_msg.size() == 0)
                        {
                                ros::spinOnce();
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                                std::this_thread::yield();
                        }
                        sensor_msgs::CompressedImageConstPtr msg = g_received_compressed_img_msg.front();
                        try
                        {
                                cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                                img_rec_time = msg->header.stamp.toSec();
                                image_get = cv_ptr_compressed->image;
                                cv_ptr_compressed->image.release();
                        }
                        catch (cv_bridge::Exception &e)
                        {
                                printf("Could not convert from '%s' to 'bgr8' !!! ", msg->format.c_str());
                        }
                        mutex_image_callback.lock();
                        g_received_compressed_img_msg.pop_front();
                        mutex_image_callback.unlock();
                }
                else
                {
                        while (g_received_img_msg.size() == 0)
                        {
                                ros::spinOnce();
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                                std::this_thread::yield();
                        }
                        sensor_msgs::ImageConstPtr msg = g_received_img_msg.front();
                        image_get = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone(); // opencv格式的图像
                        img_rec_time = msg->header.stamp.toSec();                                                //时间戳
                        mutex_image_callback.lock();
                        g_received_img_msg.pop_front(); //删除image中的第一个元素
                        mutex_image_callback.unlock();
                }
                // cout << "image_time " << img_rec_time << endl;
                process_image(image_get, img_rec_time);
        }
}

int num_comp = 0;
void R3LIVE::image_comp_callback(const sensor_msgs::CompressedImageConstPtr &msg)
{
        num_comp++;
        // if (num_comp % 4 == 0)
        if (1)
        {
                // num_comp = 0;
                // cout << "进入comp" << endl;
                std::unique_lock<std::mutex>
                    lock2(mutex_image_callback);
                if (sub_image_typed == 1)
                {
                        return; // Avoid subscribe the same image twice.
                }
                sub_image_typed = 2;
                g_received_compressed_img_msg.push_back(msg);
                if (g_flag_if_first_rec_img)
                {
                        g_flag_if_first_rec_img = 0;
                        // cout << "在这" << endl;
                        m_thread_pool_ptr->commit_task(&R3LIVE::service_process_img_buffer, this); //进入处理vio的循环
                }
                return;
        }
}

int image_number = 0;
// ANCHOR - image_callback
void R3LIVE::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
        image_number++;
        if (image_number >= 3)
                image_number = 0;
        if (1)
        {
                cout << "收到图像" << endl;
                std::unique_lock<std::mutex> lock(mutex_image_callback);
                if (sub_image_typed == 2)
                {
                        return; // Avoid subscribe the same image twice.
                }
                sub_image_typed = 1;

                if (g_flag_if_first_rec_img)
                {
                        g_flag_if_first_rec_img = 0;
                        m_thread_pool_ptr->commit_task(&R3LIVE::service_process_img_buffer, this);
                }

                cv::Mat temp_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
                process_image(temp_img, msg->header.stamp.toSec());
        }
}

double last_accept_time = 0;
int buffer_max_frame = 0;
int total_frame_count = 0;
void R3LIVE::process_image(cv::Mat &temp_img, double msg_time)
{
        // cout << "处理图像" << endl;
        cv::Mat img_get;
        if (temp_img.rows == 0)
        {
                cout << "Process image error, image rows =0 " << endl;
                return;
        }

        if (msg_time < last_accept_time)
        {
                cout << "Error, image time revert!!" << endl;
                return;
        }

        if ((msg_time - last_accept_time) < (1.0 / m_control_image_freq) * 0.9)
        {
                return;
        }
        last_accept_time = msg_time;

        if (m_camera_start_ros_tim < 0) // VIO初始化
        {
                // cout << "111111" << endl;
                m_camera_start_ros_tim = msg_time;
                /*
                m_image_downsample_ratio = 1
                */
                m_vio_scale_factor = m_vio_image_width * m_image_downsample_ratio / temp_img.cols; // 320 * 24
                cout << "m_vio_scale_factor:  " << m_vio_scale_factor << endl;
                // load_vio_parameters();
                //设置VIO参数
                set_initial_camera_parameter(g_lio_state, m_camera_intrinsic.data(), m_camera_dist_coeffs.data(), m_camera_ext_R.data(),
                                             m_camera_ext_t.data(), m_vio_scale_factor);
                cv::eigen2cv(g_cam_K, intrinsic);
                cv::eigen2cv(g_cam_dist, dist_coeffs); // Eigen转化为cv
                //输出原始图像与去畸变图像的映射关系
                initUndistortRectifyMap(intrinsic, dist_coeffs, cv::Mat(), intrinsic, cv::Size(m_vio_image_width / m_vio_scale_factor, m_vio_image_heigh / m_vio_scale_factor),
                                        CV_16SC2, m_ud_map1, m_ud_map2); //图像去畸变   通过initUndistortRectifyMap()和remap()的组合来处理
                m_thread_pool_ptr->commit_task(&R3LIVE::service_pub_rgb_maps, this);
                m_thread_pool_ptr->commit_task(&R3LIVE::service_VIO_update, this);
                m_mvs_recorder.init(g_cam_K, m_vio_image_width / m_vio_scale_factor, &m_map_rgb_pts);
                m_mvs_recorder.set_working_dir(m_map_output_dir);
        }

        if (m_image_downsample_ratio != 1.0) //图像不缩放
        {
                cv::resize(temp_img, img_get, cv::Size(m_vio_image_width / m_vio_scale_factor, m_vio_image_heigh / m_vio_scale_factor));
        }
        else
        {
                img_get = temp_img; // clone ?
        }
        std::shared_ptr<Image_frame> img_pose = std::make_shared<Image_frame>(g_cam_K); // std::make_shared< FileTest >(a, b, c):
                                                                                        //泛型参数代表的是要构建的指针类型,而a,b,c是传递给构造函数的参数
        if (m_if_pub_raw_img)
        {
                img_pose->m_raw_img = img_get;
        }
        cv::remap(img_get, img_pose->m_img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR); //图像畸变矫正 img_get:输入图像；img_pose->m_img 输出图像
        // cv::imshow("sub Img", img_pose->m_img);
        img_pose->m_timestamp = msg_time;
        img_pose->init_cubic_interpolation(); //彩色图像转为灰度图
        img_pose->image_equalize();           //对灰度图及彩色图进行直方图均衡化
        m_camera_data_mutex.lock();
        m_queue_image_with_pose.push_back(img_pose); //等待vio处理的img_pose的队列
        m_camera_data_mutex.unlock();
        total_frame_count++;

        if (m_queue_image_with_pose.size() > buffer_max_frame)
        {
                buffer_max_frame = m_queue_image_with_pose.size(); // m_queue_image_with_pose 保存等待vio处理的图像队列
        }

        // cout << "Image queue size = " << m_queue_image_with_pose.size() << endl;
}

void R3LIVE::load_vio_parameters()
{

        std::vector<double> camera_intrinsic_data, camera_dist_coeffs_data, camera_ext_R_data, camera_ext_t_data;
        m_ros_node_handle.getParam("r3live_vio/image_width", m_vio_image_width);
        m_ros_node_handle.getParam("r3live_vio/image_height", m_vio_image_heigh);
        m_ros_node_handle.getParam("r3live_vio/camera_intrinsic", camera_intrinsic_data);
        m_ros_node_handle.getParam("r3live_vio/camera_dist_coeffs", camera_dist_coeffs_data);
        m_ros_node_handle.getParam("r3live_vio/camera_ext_R", camera_ext_R_data);
        m_ros_node_handle.getParam("r3live_vio/camera_ext_t", camera_ext_t_data);

        CV_Assert((m_vio_image_width != 0 && m_vio_image_heigh != 0));

        if ((camera_intrinsic_data.size() != 9) || (camera_dist_coeffs_data.size() != 5) || (camera_ext_R_data.size() != 9) ||
            (camera_ext_t_data.size() != 3))
        {

                cout << ANSI_COLOR_RED_BOLD << "Load VIO parameter fail!!!, please check!!!" << endl;
                printf("Load camera data size = %d, %d, %d, %d\n", (int)camera_intrinsic_data.size(), camera_dist_coeffs_data.size(),
                       camera_ext_R_data.size(), camera_ext_t_data.size());
                cout << ANSI_COLOR_RESET << endl;
                std::this_thread::sleep_for(std::chrono::seconds(3000000));
        }

        m_camera_intrinsic = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_intrinsic_data.data());
        m_camera_dist_coeffs = Eigen::Map<Eigen::Matrix<double, 5, 1>>(camera_dist_coeffs_data.data());
        m_camera_ext_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_ext_R_data.data());
        m_camera_ext_t = Eigen::Map<Eigen::Matrix<double, 3, 1>>(camera_ext_t_data.data());

        cout << "[Ros_parameter]: r3live_vio/Camera Intrinsic: " << endl;
        cout << m_camera_intrinsic << endl;
        cout << "[Ros_parameter]: r3live_vio/Camera distcoeff: " << m_camera_dist_coeffs.transpose() << endl;
        cout << "[Ros_parameter]: r3live_vio/Camera extrinsic R: " << endl;
        cout << m_camera_ext_R << endl;
        cout << "[Ros_parameter]: r3live_vio/Camera extrinsic T: " << m_camera_ext_t.transpose() << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
}

void R3LIVE::set_image_pose(std::shared_ptr<Image_frame> &image_pose, const StatesGroup &state)
{
        mat_3_3 rot_mat = state.rot_end;
        vec_3 t_vec = state.pos_end;
        vec_3 pose_t = rot_mat * state.pos_ext_i2c + t_vec; //计算出相机相对于世界坐标系下的位姿
        mat_3_3 R_w2c = rot_mat * state.rot_ext_i2c;

        image_pose->set_pose(eigen_q(R_w2c), pose_t);
        image_pose->fx = state.cam_intrinsic(0);
        image_pose->fy = state.cam_intrinsic(1);
        image_pose->cx = state.cam_intrinsic(2);
        image_pose->cy = state.cam_intrinsic(3);

        image_pose->m_cam_K << image_pose->fx, 0, image_pose->cx, 0, image_pose->fy, image_pose->cy, 0, 0, 1;
        scope_color(ANSI_COLOR_CYAN_BOLD);
        // cout << "Set Image Pose frm [" << image_pose->m_frame_idx << "], pose: " << eigen_q(rot_mat).coeffs().transpose()
        // << " | " << t_vec.transpose()
        // << " | " << eigen_q(rot_mat).angularDistance( eigen_q::Identity()) *57.3 << endl;
        // image_pose->inverse_pose();
}

void R3LIVE::publish_camera_odom(std::shared_ptr<Image_frame> &image, double msg_time)
{
        eigen_q odom_q = image->m_pose_w2c_q;
        vec_3 odom_t = image->m_pose_w2c_t;
        nav_msgs::Odometry camera_odom;
        camera_odom.header.frame_id = "world";
        camera_odom.child_frame_id = "/aft_mapped";
        camera_odom.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
        camera_odom.pose.pose.orientation.x = odom_q.x();
        camera_odom.pose.pose.orientation.y = odom_q.y();
        camera_odom.pose.pose.orientation.z = odom_q.z();
        camera_odom.pose.pose.orientation.w = odom_q.w();
        camera_odom.pose.pose.position.x = odom_t(0);
        camera_odom.pose.pose.position.y = odom_t(1);
        camera_odom.pose.pose.position.z = odom_t(2);
        pub_odom_cam.publish(camera_odom);

        geometry_msgs::PoseStamped msg_pose;
        msg_pose.header.stamp = ros::Time().fromSec(msg_time);
        msg_pose.header.frame_id = "world";
        msg_pose.pose.orientation.x = odom_q.x();
        msg_pose.pose.orientation.y = odom_q.y();
        msg_pose.pose.orientation.z = odom_q.z();
        msg_pose.pose.orientation.w = odom_q.w();
        msg_pose.pose.position.x = odom_t(0);
        msg_pose.pose.position.y = odom_t(1);
        msg_pose.pose.position.z = odom_t(2);
        camera_path.header.frame_id = "world";
        camera_path.poses.push_back(msg_pose);
        pub_path_cam.publish(camera_path);
}

void R3LIVE::publish_track_pts(Rgbmap_tracker &tracker)
{
        pcl::PointXYZRGB temp_point;
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_for_pub;

        for (auto it : tracker.m_map_rgb_pts_in_current_frame_pos)
        {
                vec_3 pt = ((RGB_pts *)it.first)->get_pos();
                cv::Scalar color = ((RGB_pts *)it.first)->m_dbg_color;
                temp_point.x = pt(0);
                temp_point.y = pt(1);
                temp_point.z = pt(2);
                temp_point.r = color(2);
                temp_point.g = color(1);
                temp_point.b = color(0);
                pointcloud_for_pub.points.push_back(temp_point);
        }
        sensor_msgs::PointCloud2 ros_pc_msg;
        pcl::toROSMsg(pointcloud_for_pub, ros_pc_msg);
        ros_pc_msg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
        ros_pc_msg.header.frame_id = "world";       // world; camera_init
        m_pub_visual_tracked_3d_pts.publish(ros_pc_msg);
}

// ANCHOR - VIO preintegration
bool R3LIVE::vio_preintegration(StatesGroup &state_in, StatesGroup &state_out, double current_frame_time)
{
        state_out = state_in;
        if (current_frame_time <= state_in.last_update_time)
        {
                // cout << ANSI_COLOR_RED_BOLD << "Error current_frame_time <= state_in.last_update_time | " <<
                // current_frame_time - state_in.last_update_time << ANSI_COLOR_RESET << endl;
                return false;
        }
        mtx_buffer.lock();
        std::deque<sensor_msgs::Imu::ConstPtr> vio_imu_queue;
        for (auto it = imu_buffer_vio.begin(); it != imu_buffer_vio.end(); it++)
        {
                vio_imu_queue.push_back(*it); //取出IMU数据，但是时间不能大于当前图像帧的时间

                if ((*it)->header.stamp.toSec() > current_frame_time)
                {
                        break;
                }
        }

        while (!imu_buffer_vio.empty())
        {
                double imu_time = imu_buffer_vio.front()->header.stamp.toSec();
                if (imu_time < current_frame_time - 0.2)
                {
                        imu_buffer_vio.pop_front(); //删除容器里面第一个元素
                }
                else
                {
                        break;
                }
        }
        // cout << "Current VIO_imu buffer size = " << imu_buffer_vio.size() << endl;
        state_out = m_imu_process->imu_preintegration(state_out, vio_imu_queue, current_frame_time - vio_imu_queue.back()->header.stamp.toSec());
        eigen_q q_diff(state_out.rot_end.transpose() * state_in.rot_end);
        // cout << "Pos diff = " << (state_out.pos_end - state_in.pos_end).transpose() << endl;
        // cout << "Euler diff = " << q_diff.angularDistance(eigen_q::Identity()) * 57.3 << endl;
        mtx_buffer.unlock();
        state_out.last_update_time = current_frame_time;
        return true;
}

// ANCHOR - huber_loss
double get_huber_loss_scale(double reprojection_error, double outlier_threshold = 1.0)
{
        // http://ceres-solver.org/nnls_modeling.html#lossfunction
        double scale = 1.0;
        if (reprojection_error / outlier_threshold < 1.0)
        {
                scale = 1.0;
        }
        else
        {
                scale = (2 * sqrt(reprojection_error) / sqrt(outlier_threshold) - 1.0) / reprojection_error;
        }
        return scale;
}

// ANCHOR - VIO_esikf
const int minimum_iteration_pts = 10;
bool R3LIVE::vio_esikf(StatesGroup &state_in, Rgbmap_tracker &op_track)
{
        Common_tools::Timer tim;
        tim.tic();
        scope_color(ANSI_COLOR_BLUE_BOLD);
        StatesGroup state_iter = state_in;
        if (!m_if_estimate_intrinsic) // When disable the online intrinsic calibration.
        {
                state_iter.cam_intrinsic << g_cam_K(0, 0), g_cam_K(1, 1), g_cam_K(0, 2), g_cam_K(1, 2);
        }

        if (!m_if_estimate_i2c_extrinsic)
        {
                state_iter.pos_ext_i2c = m_inital_pos_ext_i2c;
                state_iter.rot_ext_i2c = m_inital_rot_ext_i2c;
        }

        Eigen::Matrix<double, -1, -1> H_mat;   // H
        Eigen::Matrix<double, -1, 1> meas_vec; //残差
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
        Eigen::Matrix<double, DIM_OF_STATES, 1> solution, solution1;
        Eigen::Matrix<double, -1, -1> K, KH;
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> K_1;

        Eigen::SparseMatrix<double> H_mat_spa, H_T_H_spa, K_spa, KH_spa, vec_spa, I_STATE_spa; //稀疏矩阵（把矩阵里面的0去掉）
        I_STATE.setIdentity();
        I_STATE_spa = I_STATE.sparseView();
        double fx, fy, cx, cy, time_td;

        int total_pt_size = op_track.m_map_rgb_pts_in_current_frame_pos.size(); //当前帧的特征点
        std::vector<double> last_reprojection_error_vec(total_pt_size), current_reprojection_error_vec(total_pt_size);

        if (total_pt_size < minimum_iteration_pts) //如果特征点小于10，不更新vio
        {
                state_in = state_iter;
                return false;
        }

        H_mat.resize(total_pt_size * 2, DIM_OF_STATES);
        meas_vec.resize(total_pt_size * 2, 1); //残差
        double last_repro_err = 3e8;           //重投影误差
        int avail_pt_count = 0;
        double last_avr_repro_err = 0;

        double acc_reprojection_error = 0;
        double img_res_scale = 1.0;
        for (int iter_count = 0; iter_count < esikf_iter_times; iter_count++)
        {

                // cout << "========== Iter " << iter_count << " =========" << endl;
                mat_3_3 R_imu = state_iter.rot_end;
                vec_3 t_imu = state_iter.pos_end;
                vec_3 t_c2w = R_imu * state_iter.pos_ext_i2c + t_imu;
                mat_3_3 R_c2w = R_imu * state_iter.rot_ext_i2c; // world to camera frame

                fx = state_iter.cam_intrinsic(0);
                fy = state_iter.cam_intrinsic(1);
                cx = state_iter.cam_intrinsic(2);
                cy = state_iter.cam_intrinsic(3);

                //从状态变量中取出当前迭代的时间偏差
                time_td = state_iter.td_ext_i2c_delta;

                vec_3 t_w2c = -R_c2w.transpose() * t_c2w;
                mat_3_3 R_w2c = R_c2w.transpose();
                int pt_idx = -1;
                acc_reprojection_error = 0;
                vec_3 pt_3d_w, pt_3d_cam;
                vec_2 pt_img_measure, pt_img_proj, pt_img_vel;
                eigen_mat_d<2, 3> mat_pre;
                eigen_mat_d<3, 3> mat_A, mat_B, mat_C, mat_D, pt_hat;
                H_mat.setZero();
                solution.setZero();
                solution1.setZero();
                meas_vec.setZero();
                avail_pt_count = 0;
                for (auto it = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); it != op_track.m_map_rgb_pts_in_last_frame_pos.end(); it++) //遍历上一帧的特征点，构造残差
                {
                        //地图点在世界坐标系下的3D坐标
                        pt_3d_w = ((RGB_pts *)it->first)->get_pos(); //上一帧特征点的坐标

                        //上一帧图像的特征点的运动速度
                        pt_img_vel = ((RGB_pts *)it->first)->m_img_vel;

                        //上一帧追踪到的光流与当前帧特征点
                        pt_img_measure = vec_2(it->second.x, it->second.y); //对应的当前帧特征点

                        //地图点在当前相机帧下观测的3D坐标
                        pt_3d_cam = R_w2c * pt_3d_w + t_w2c; //上一帧特征点的坐标投影到当前帧

                        //考虑时间矫正，地图点在当前帧下的2D坐标
                        pt_img_proj = vec_2(fx * pt_3d_cam(0) / pt_3d_cam(2) + cx, fy * pt_3d_cam(1) / pt_3d_cam(2) + cy) + time_td * pt_img_vel;

                        double repro_err = (pt_img_proj - pt_img_measure).norm();  //重投影误差
                        double huber_loss_scale = get_huber_loss_scale(repro_err); //
                        pt_idx++;
                        acc_reprojection_error += repro_err;
                        // if (iter_count == 0 || ((repro_err - last_reprojection_error_vec[pt_idx]) < 1.5))
                        if (iter_count == 0 || ((repro_err - last_avr_repro_err * 5.0) < 0))
                        {
                                last_reprojection_error_vec[pt_idx] = repro_err;
                        }
                        else
                        {
                                last_reprojection_error_vec[pt_idx] = repro_err;
                        }
                        avail_pt_count++;
                        // Appendix E of r2live_Supplementary_material.
                        // https://github.com/hku-mars/r2live/blob/master/supply/r2live_Supplementary_material.pdf
                        //计算雅克比
                        mat_pre << fx / pt_3d_cam(2), 0, -fx * pt_3d_cam(0) / pt_3d_cam(2), 0, fy / pt_3d_cam(2), -fy * pt_3d_cam(1) / pt_3d_cam(2);

                        pt_hat = Sophus::SO3d::hat((R_imu.transpose() * (pt_3d_w - t_imu)));
                        // Eigen::Vector3d eulerAngle = R_imu.matrix().eulerAngles(2, 1, 0);
                        // cout << "R_imu: " << eulerAngle.transpose() * 57.3 << endl;

                        mat_A = state_iter.rot_ext_i2c.transpose() * pt_hat;
                        mat_B = -state_iter.rot_ext_i2c.transpose() * (R_imu.transpose());
                        mat_C = Sophus::SO3d::hat(pt_3d_cam);
                        mat_D = -state_iter.rot_ext_i2c.transpose();
                        meas_vec.block(pt_idx * 2, 0, 2, 1) = (pt_img_proj - pt_img_measure) * huber_loss_scale / img_res_scale;

                        H_mat.block(pt_idx * 2, 0, 2, 3) = mat_pre * mat_A * huber_loss_scale;
                        H_mat.block(pt_idx * 2, 3, 2, 3) = mat_pre * mat_B * huber_loss_scale;

                        //估计时间偏移
                        if (DIM_OF_STATES > 24)
                        {
                                // Estimate time td.
                                H_mat.block(pt_idx * 2, 24, 2, 1) = pt_img_vel * huber_loss_scale;
                                // H_mat(pt_idx * 2, 24) = pt_img_vel(0) * huber_loss_scale;
                                // H_mat(pt_idx * 2 + 1, 24) = pt_img_vel(1) * huber_loss_scale;
                        }
                        if (m_if_estimate_i2c_extrinsic) //是否估计相机 IMU的外参
                        {
                                H_mat.block(pt_idx * 2, 18, 2, 3) = mat_pre * mat_C * huber_loss_scale;
                                H_mat.block(pt_idx * 2, 21, 2, 3) = mat_pre * mat_D * huber_loss_scale;
                        }

                        if (m_if_estimate_intrinsic) //是否估计相机内参
                        {
                                H_mat(pt_idx * 2, 25) = pt_3d_cam(0) / pt_3d_cam(2) * huber_loss_scale;
                                H_mat(pt_idx * 2 + 1, 26) = pt_3d_cam(1) / pt_3d_cam(2) * huber_loss_scale;
                                H_mat(pt_idx * 2, 27) = 1 * huber_loss_scale;
                                H_mat(pt_idx * 2 + 1, 28) = 1 * huber_loss_scale;
                        }
                }
                H_mat = H_mat / img_res_scale;
                acc_reprojection_error /= total_pt_size;

                last_avr_repro_err = acc_reprojection_error;
                if (avail_pt_count < minimum_iteration_pts)
                {
                        break;
                }

                H_mat_spa = H_mat.sparseView();

                Eigen::SparseMatrix<double> Hsub_T_temp_mat = H_mat_spa.transpose();

                vec_spa = (state_iter - state_in).sparseView();

                // cout << "qian " << solution.rot_end << end;
                H_T_H_spa = Hsub_T_temp_mat * H_mat_spa;

                // Notice that we have combine some matrix using () in order to boost the matrix multiplication.
                Eigen::SparseMatrix<double> temp_inv_mat = //(H^TH + P^-1)^-1
                    ((H_T_H_spa.toDense() + eigen_mat<-1, -1>(state_in.cov * m_cam_measurement_weight).inverse()).inverse()).sparseView();

                KH_spa = temp_inv_mat * (Hsub_T_temp_mat * H_mat_spa); // KH

                solution = (temp_inv_mat * (Hsub_T_temp_mat * (-1 * meas_vec.sparseView())) - (I_STATE_spa - KH_spa) * vec_spa).toDense();

                solution1 = (temp_inv_mat * Hsub_T_temp_mat * (1 * meas_vec.sparseView() - H_mat_spa * vec_spa)).toDense();

                // cout_solution = solution;
                // cout_solution1 = solution1;
                //  Eigen::Matrix<double, DIM_OF_STATES, 1> ss = solution;
                //   ROS_INFO("111", solution);

                // cout << "solution1 " << ss << end;

                state_iter = state_iter + solution;

                // cout << "solution1 " << state_iter.rot_end << end;

                if (fabs(acc_reprojection_error - last_repro_err) < 0.01)
                {
                        break;
                }
                last_repro_err = acc_reprojection_error;
        }

        if (avail_pt_count >= minimum_iteration_pts)
        {
                state_iter.cov = ((I_STATE_spa - KH_spa) * state_iter.cov.sparseView()).toDense();
        }

        state_iter.td_ext_i2c += state_iter.td_ext_i2c_delta;
        state_iter.td_ext_i2c_delta = 0;
        state_in = state_iter;
        return true;
}

bool R3LIVE::vio_photometric(StatesGroup &state_in, Rgbmap_tracker &op_track, std::shared_ptr<Image_frame> &image)
{
        Common_tools::Timer tim;
        tim.tic();
        StatesGroup state_iter = state_in;
        if (!m_if_estimate_intrinsic) // When disable the online intrinsic calibration.
        {
                state_iter.cam_intrinsic << g_cam_K(0, 0), g_cam_K(1, 1), g_cam_K(0, 2), g_cam_K(1, 2);
        }
        if (!m_if_estimate_i2c_extrinsic) // When disable the online extrinsic calibration.
        {
                state_iter.pos_ext_i2c = m_inital_pos_ext_i2c;
                state_iter.rot_ext_i2c = m_inital_rot_ext_i2c;
        }
        Eigen::Matrix<double, -1, -1> H_mat, R_mat_inv;
        Eigen::Matrix<double, -1, 1> meas_vec;
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
        Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
        Eigen::Matrix<double, -1, -1> K, KH;
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> K_1;
        Eigen::SparseMatrix<double> H_mat_spa, H_T_H_spa, R_mat_inv_spa, K_spa, KH_spa, vec_spa, I_STATE_spa;
        I_STATE.setIdentity();
        I_STATE_spa = I_STATE.sparseView();
        double fx, fy, cx, cy, time_td;

        int total_pt_size = op_track.m_map_rgb_pts_in_current_frame_pos.size();
        std::vector<double> last_reprojection_error_vec(total_pt_size), current_reprojection_error_vec(total_pt_size);
        if (total_pt_size < minimum_iteration_pts)
        {
                state_in = state_iter;
                return false;
        }

        int err_size = 3;
        H_mat.resize(total_pt_size * err_size, DIM_OF_STATES);
        meas_vec.resize(total_pt_size * err_size, 1);
        R_mat_inv.resize(total_pt_size * err_size, total_pt_size * err_size);

        double last_repro_err = 3e8;
        int avail_pt_count = 0;
        double last_avr_repro_err = 0;
        int if_esikf = 1;

        double acc_photometric_error = 0;
        for (int iter_count = 0; iter_count < 2; iter_count++)
        {
                mat_3_3 R_imu = state_iter.rot_end;
                vec_3 t_imu = state_iter.pos_end;
                vec_3 t_c2w = R_imu * state_iter.pos_ext_i2c + t_imu;
                mat_3_3 R_c2w = R_imu * state_iter.rot_ext_i2c; // world to camera frame

                fx = state_iter.cam_intrinsic(0);
                fy = state_iter.cam_intrinsic(1);
                cx = state_iter.cam_intrinsic(2);
                cy = state_iter.cam_intrinsic(3);
                time_td = state_iter.td_ext_i2c_delta;

                vec_3 t_w2c = -R_c2w.transpose() * t_c2w;
                mat_3_3 R_w2c = R_c2w.transpose();
                int pt_idx = -1;
                acc_photometric_error = 0;
                vec_3 pt_3d_w, pt_3d_cam;
                vec_2 pt_img_measure, pt_img_proj, pt_img_vel;
                eigen_mat_d<2, 3> mat_pre;
                eigen_mat_d<3, 2> mat_photometric;
                eigen_mat_d<3, 3> mat_d_pho_d_img;
                eigen_mat_d<3, 3> mat_A, mat_B, mat_C, mat_D, pt_hat;
                R_mat_inv.setZero();
                H_mat.setZero();
                solution.setZero();
                meas_vec.setZero();
                avail_pt_count = 0;
                int iter_layer = 0;
                tim.tic("Build_cost");
                for (auto it = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); it != op_track.m_map_rgb_pts_in_last_frame_pos.end(); it++)
                {
                        if (((RGB_pts *)it->first)->m_N_rgb < 3)
                        {
                                continue;
                        }
                        pt_idx++;
                        pt_3d_w = ((RGB_pts *)it->first)->get_pos();
                        pt_img_vel = ((RGB_pts *)it->first)->m_img_vel;
                        pt_img_measure = vec_2(it->second.x, it->second.y);
                        pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
                        pt_img_proj = vec_2(fx * pt_3d_cam(0) / pt_3d_cam(2) + cx, fy * pt_3d_cam(1) / pt_3d_cam(2) + cy) + time_td * pt_img_vel;

                        vec_3 pt_rgb = ((RGB_pts *)it->first)->get_rgb();
                        mat_3_3 pt_rgb_info = mat_3_3::Zero();
                        mat_3_3 pt_rgb_cov = ((RGB_pts *)it->first)->get_rgb_cov();
                        for (int i = 0; i < 3; i++)
                        {
                                pt_rgb_info(i, i) = 1.0 / pt_rgb_cov(i, i) / 255;
                                R_mat_inv(pt_idx * err_size + i, pt_idx * err_size + i) = pt_rgb_info(i, i);
                                // R_mat_inv( pt_idx * err_size + i, pt_idx * err_size + i ) =  1.0;
                        }
                        vec_3 obs_rgb_dx, obs_rgb_dy;
                        vec_3 obs_rgb = image->get_rgb(pt_img_proj(0), pt_img_proj(1), 0, &obs_rgb_dx, &obs_rgb_dy);
                        vec_3 photometric_err_vec = (obs_rgb - pt_rgb);
                        double huber_loss_scale = get_huber_loss_scale(photometric_err_vec.norm());
                        photometric_err_vec *= huber_loss_scale;
                        double photometric_err = photometric_err_vec.transpose() * pt_rgb_info * photometric_err_vec;

                        acc_photometric_error += photometric_err;

                        last_reprojection_error_vec[pt_idx] = photometric_err;

                        avail_pt_count++;
                        mat_pre << fx / pt_3d_cam(2), 0, -fx * pt_3d_cam(0) / pt_3d_cam(2), 0, fy / pt_3d_cam(2), -fy * pt_3d_cam(1) / pt_3d_cam(2);
                        mat_d_pho_d_img = mat_photometric * mat_pre;

                        pt_hat = Sophus::SO3d::hat((R_imu.transpose() * (pt_3d_w - t_imu)));
                        mat_A = state_iter.rot_ext_i2c.transpose() * pt_hat;
                        mat_B = -state_iter.rot_ext_i2c.transpose() * (R_imu.transpose());
                        mat_C = Sophus::SO3d::hat(pt_3d_cam);
                        mat_D = -state_iter.rot_ext_i2c.transpose();
                        meas_vec.block(pt_idx * 3, 0, 3, 1) = photometric_err_vec;

                        H_mat.block(pt_idx * 3, 0, 3, 3) = mat_d_pho_d_img * mat_A * huber_loss_scale;
                        H_mat.block(pt_idx * 3, 3, 3, 3) = mat_d_pho_d_img * mat_B * huber_loss_scale;
                        if (1)
                        {
                                if (m_if_estimate_i2c_extrinsic)
                                {
                                        H_mat.block(pt_idx * 3, 18, 3, 3) = mat_d_pho_d_img * mat_C * huber_loss_scale;
                                        H_mat.block(pt_idx * 3, 21, 3, 3) = mat_d_pho_d_img * mat_D * huber_loss_scale;
                                }
                        }
                }
                R_mat_inv_spa = R_mat_inv.sparseView();

                last_avr_repro_err = acc_photometric_error;
                if (avail_pt_count < minimum_iteration_pts)
                {
                        break;
                }
                // Esikf
                tim.tic("Iter");
                if (if_esikf)
                {
                        H_mat_spa = H_mat.sparseView();
                        Eigen::SparseMatrix<double> Hsub_T_temp_mat = H_mat_spa.transpose();
                        vec_spa = (state_iter - state_in).sparseView();
                        H_T_H_spa = Hsub_T_temp_mat * R_mat_inv_spa * H_mat_spa;
                        Eigen::SparseMatrix<double> temp_inv_mat =
                            (H_T_H_spa.toDense() + (state_in.cov * m_cam_measurement_weight).inverse()).inverse().sparseView();
                        Eigen::SparseMatrix<double> Ht_R_inv = (Hsub_T_temp_mat * R_mat_inv_spa);
                        KH_spa = Ht_R_inv * H_mat_spa;
                        // cout << "Ht_R_inv:  " << Ht_R_inv << endl;
                        // cout << "H_mat_spa:  " << H_mat_spa << endl;
                        solution =
                            (temp_inv_mat * (Ht_R_inv * ((-1 * meas_vec.sparseView()))) - (I_STATE_spa - temp_inv_mat * KH_spa) * vec_spa).toDense();
                }
                // cout << "solution " << solution << endl;
                state_iter = state_iter + solution;
                // state_iter.cov = ((I_STATE_spa - KH_spa) * state_iter.cov.sparseView()).toDense();
                if ((acc_photometric_error / total_pt_size) < 10) // By experience.
                {
                        break;
                }
                if (fabs(acc_photometric_error - last_repro_err) < 0.01)
                {
                        break;
                }
                last_repro_err = acc_photometric_error;
        }
        if (if_esikf && avail_pt_count >= minimum_iteration_pts)
        {
                state_iter.cov = ((I_STATE_spa - KH_spa) * state_iter.cov.sparseView()).toDense();
        }
        state_iter.td_ext_i2c += state_iter.td_ext_i2c_delta;
        state_iter.td_ext_i2c_delta = 0;
        state_in = state_iter;
        return true;
}

void R3LIVE::service_pub_rgb_maps()
{
        int last_publish_map_idx = -3e8;
        int sleep_time_aft_pub = 10;
        int number_of_pts_per_topic = 1000;
        if (number_of_pts_per_topic < 0)
        {
                return;
        }
        while (1)
        {
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                pcl::PointCloud<pcl::PointXYZRGB> pc_rgb; // 发布pcl点云地图
                sensor_msgs::PointCloud2 ros_pc_msg;
                int pts_size = m_map_rgb_pts.m_rgb_pts_vec.size(); // Global_map m_map_rgb_pts全局地图
                // cout << "pts_size  " << pts_size << endl;
                pc_rgb.resize(number_of_pts_per_topic); //定义点云地图的尺寸
                // for (int i = pts_size - 1; i > 0; i--)
                int pub_idx_size = 0;
                int cur_topic_idx = 0;
                if (last_publish_map_idx == m_map_rgb_pts.m_last_updated_frame_idx)
                {
                        continue;
                }
                last_publish_map_idx = m_map_rgb_pts.m_last_updated_frame_idx;
                // cout << "pts_size " << pts_size << endl;
                for (int i = 0; i < pts_size; i++)
                {
                        if (m_map_rgb_pts.m_rgb_pts_vec[i]->m_N_rgb < 1)
                        {
                                continue;
                        }
                        pc_rgb.points[pub_idx_size].x = m_map_rgb_pts.m_rgb_pts_vec[i]->m_pos[0];
                        pc_rgb.points[pub_idx_size].y = m_map_rgb_pts.m_rgb_pts_vec[i]->m_pos[1];
                        pc_rgb.points[pub_idx_size].z = m_map_rgb_pts.m_rgb_pts_vec[i]->m_pos[2];
                        pc_rgb.points[pub_idx_size].r = m_map_rgb_pts.m_rgb_pts_vec[i]->m_rgb[2];
                        pc_rgb.points[pub_idx_size].g = m_map_rgb_pts.m_rgb_pts_vec[i]->m_rgb[1];
                        pc_rgb.points[pub_idx_size].b = m_map_rgb_pts.m_rgb_pts_vec[i]->m_rgb[0];
                        // pc_rgb.points[i].intensity = m_map_rgb_pts.m_rgb_pts_vec[i]->m_obs_dis;
                        pub_idx_size++;
                        if (pub_idx_size == number_of_pts_per_topic) //如果发布出去的点云等于子图的最大尺寸
                        {
                                pub_idx_size = 0;
                                pcl::toROSMsg(pc_rgb, ros_pc_msg); //点云地图发布
                                ros_pc_msg.header.frame_id = "world";
                                ros_pc_msg.header.stamp = ros::Time::now();                        // cur_topic_idx ： 当前子图
                                if (m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] == nullptr) // m_pub_rgb_render_pointcloud_ptr_vec 发布点云地图
                                {
                                        m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] =
                                            std::make_shared<ros::Publisher>(m_ros_node_handle.advertise<sensor_msgs::PointCloud2>(
                                                std::string("/RGB_map_").append(std::to_string(cur_topic_idx)), 100)); // std::make_shared智能指针
                                        // m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>
                                }
                                m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx]->publish(ros_pc_msg);
                                std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_aft_pub));
                                ros::spinOnce();
                                cur_topic_idx++;
                        }
                }
                // std::cout << "pub_idx_size  " << pub_idx_size << endl;
                pc_rgb.resize(pub_idx_size);
                pcl::toROSMsg(pc_rgb, ros_pc_msg);
                ros_pc_msg.header.frame_id = "world";
                ros_pc_msg.header.stamp = ros::Time::now();
                if (m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] == nullptr)
                {
                        m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] =
                            std::make_shared<ros::Publisher>(m_ros_node_handle.advertise<sensor_msgs::PointCloud2>(
                                std::string("/RGB_map_").append(std::to_string(cur_topic_idx)), 100));
                }
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_aft_pub)); //线程阻塞的时间
                ros::spinOnce();
                m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx]->publish(ros_pc_msg);
                cur_topic_idx++;
                if (cur_topic_idx >= 45) // Maximum pointcloud topics = 45.
                {
                        number_of_pts_per_topic *= 1.5;
                        sleep_time_aft_pub *= 1.5;
                }
                // cout << "cur_topic_idx " << cur_topic_idx << endl;
                // cout << "number_of_pts_per_topic " << number_of_pts_per_topic << endl;
                // cout << "sleep_time_aft_pub " << sleep_time_aft_pub << endl;
        }
}

void R3LIVE::publish_render_pts(ros::Publisher &pts_pub, Global_map &m_map_rgb_pts)
{
        pcl::PointCloud<pcl::PointXYZRGB> pc_rgb;
        sensor_msgs::PointCloud2 ros_pc_msg;
        pc_rgb.reserve(1e7);
        m_map_rgb_pts.m_mutex_m_box_recent_hitted->lock();
        std::unordered_set<std::shared_ptr<RGB_Voxel>> boxes_recent_hitted = m_map_rgb_pts.m_voxels_recent_visited;
        m_map_rgb_pts.m_mutex_m_box_recent_hitted->unlock();

        for (Voxel_set_iterator it = boxes_recent_hitted.begin(); it != boxes_recent_hitted.end(); it++)
        {
                for (int pt_idx = 0; pt_idx < (*it)->m_pts_in_grid.size(); pt_idx++)
                {
                        pcl::PointXYZRGB pt;
                        std::shared_ptr<RGB_pts> rgb_pt = (*it)->m_pts_in_grid[pt_idx];
                        pt.x = rgb_pt->m_pos[0];
                        pt.y = rgb_pt->m_pos[1];
                        pt.z = rgb_pt->m_pos[2];
                        pt.r = rgb_pt->m_rgb[2];
                        pt.g = rgb_pt->m_rgb[1];
                        pt.b = rgb_pt->m_rgb[0];
                        if (rgb_pt->m_N_rgb > m_pub_pt_minimum_views)
                        {
                                pc_rgb.points.push_back(pt);
                        }
                }
        }
        pcl::toROSMsg(pc_rgb, ros_pc_msg);
        ros_pc_msg.header.frame_id = "world";       // world; camera_init
        ros_pc_msg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
        pts_pub.publish(ros_pc_msg);
}

char R3LIVE::cv_keyboard_callback()
{
        char c = cv_wait_key(1);
        // return c;
        if (c == 's' || c == 'S')
        {
                scope_color(ANSI_COLOR_GREEN_BOLD);
                cout << "I capture the keyboard input!!!" << endl;
                m_mvs_recorder.export_to_mvs(m_map_rgb_pts);
                // m_map_rgb_pts.save_and_display_pointcloud( m_map_output_dir, std::string("/rgb_pt"), std::max(m_pub_pt_minimum_views, 5) );
                m_map_rgb_pts.save_and_display_pointcloud(m_map_output_dir, std::string("/rgb_pt"), m_pub_pt_minimum_views);
        }
        return c;
}

// ANCHOR -  service_VIO_update
void R3LIVE::service_VIO_update()
{
        // Init cv windows for debug
        //内参及畸变参数转换为opencv存储，并计算原图与去畸变图像的像素对应关系
        op_track.set_intrinsic(g_cam_K, g_cam_dist * 0, cv::Size(m_vio_image_width / m_vio_scale_factor, m_vio_image_heigh / m_vio_scale_factor));
        op_track.m_maximum_vio_tracked_pts = m_maximum_vio_tracked_pts;         // vio最大跟踪的特征点的数目
        m_map_rgb_pts.m_minimum_depth_for_projection = m_tracker_minimum_depth; // m_tracker_minimum_depth 0.1  跟踪特征点的最大深度
        m_map_rgb_pts.m_maximum_depth_for_projection = m_tracker_maximum_depth; // m_tracker_maximum_depth 200  跟踪特征点的最小深度
        // cv::imshow("Control panel", generate_control_panel_img().clone());

        Common_tools::Timer tim;

        cv::Mat img_get;
        while (ros::ok())
        {
                // cv_keyboard_callback();
                while (g_camera_lidar_queue.m_if_have_lidar_data == 0)
                {
                        ros::spinOnce();
                        std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_TIM));
                        std::this_thread::yield();
                        continue;
                }
                // cout << "111" << endl;
                if (m_queue_image_with_pose.size() == 0)
                {
                        ros::spinOnce();
                        std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_TIM));
                        std::this_thread::yield();
                        continue;
                }
                m_camera_data_mutex.lock();
                // cout << m_queue_image_with_pose.size()  << endl;
                while (m_queue_image_with_pose.size() > m_maximum_image_buffer) //如果待处理的图像太多，就在这个循环里面执行 m_maximum_image_buffer = 20000
                {
                        // cout << "进入循环 " << endl;
                        cout << ANSI_COLOR_BLUE_BOLD << "=== Pop image! current queue size = " << m_queue_image_with_pose.size() << " ===" << ANSI_COLOR_RESET
                             << endl;
                        op_track.track_img(m_queue_image_with_pose.front(), -20); //光流跟踪，跟踪结果也放在缓冲区里的img_pose，m_queue_image_with_pose是img_pose的队列
                        m_queue_image_with_pose.pop_front();                      //从容器中删除/弹出第一个元素
                }
                // cout << "image队列尺寸  " << m_queue_image_with_pose.size() << endl;

                std::shared_ptr<Image_frame> img_pose = m_queue_image_with_pose.front(); //第一个元素
                double message_time = img_pose->m_timestamp;
                m_queue_image_with_pose.pop_front(); //从容器中删除/弹出第一个元素
                m_camera_data_mutex.unlock();
                g_camera_lidar_queue.m_last_visual_time = img_pose->m_timestamp + g_lio_state.td_ext_i2c;

                img_pose->set_frame_idx(g_camera_frame_idx); //设置id号
                tim.tic("Frame");

                /**
                 * @brief 处理vio的第一帧图像，从lio处获取vio的起始状态
                 *
                 */
                if (g_camera_frame_idx == 0) //第一帧图像
                {
                        std::vector<cv::Point2f> pts_2d_vec;
                        std::vector<std::shared_ptr<RGB_pts>> rgb_pts_vec;
                        // while ( ( m_map_rgb_pts.is_busy() ) || ( ( m_map_rgb_pts.m_rgb_pts_vec.size() <= 100 ) ) )
                        /**
                         * @brief 等待雷达点大于100，后续要选取一些地图点投影到当前图像帧内，作为初始跟踪点来初始化光流
                         *
                         */
                        while (((m_map_rgb_pts.m_rgb_pts_vec.size() <= 100))) //
                        {
                                ros::spinOnce();
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        set_image_pose(img_pose, g_lio_state); // 对于第一帧图像的姿态，我们假定运动是静态的。

                        /**
                         * @brief 把最近访问过的体素内的满足条件的地图点，提取出来
                         * 3D点保存到rgb_pts_vec
                         * 地图点的投影保存到pts_2d_vec
                         */
                        //挑选用于重投影的特征点
                        m_map_rgb_pts.selection_points_for_projection(img_pose, &rgb_pts_vec, &pts_2d_vec, m_track_windows_size / m_vio_scale_factor);
                        op_track.init(img_pose, rgb_pts_vec, pts_2d_vec);
                        g_camera_frame_idx++;
                        continue;
                }

                g_camera_frame_idx++;
                tim.tic("Wait");
                while (g_camera_lidar_queue.if_camera_can_process() == false)
                {
                        ros::spinOnce();
                        std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_TIM));
                        std::this_thread::yield();
                        cv_keyboard_callback();
                }
                g_cost_time_logger.record(tim, "Wait");
                m_mutex_lio_process.lock();
                tim.tic("Frame");
                tim.tic("Track_img");
                StatesGroup state_out;
                m_cam_measurement_weight = std::max(0.001, std::min(5.0 / m_number_of_new_visited_voxel, 0.01));
                //处理IMU缓冲区里的数据
                if (vio_preintegration(g_lio_state, state_out, img_pose->m_timestamp + g_lio_state.td_ext_i2c) == false) //预积分
                {
                        m_mutex_lio_process.unlock();
                        continue;
                }
                set_image_pose(img_pose, state_out); //设置由IMU预积分得到的位姿初始值

                op_track.track_img(img_pose, -20); //跟踪特征点。在特征点跟踪时会先用RANSAC求基础矩阵过滤误匹配
                g_cost_time_logger.record(tim, "Track_img");
                // cout << "Track_img cost " << tim.toc( "Track_img" ) << endl;
                tim.tic("Ransac");
                set_image_pose(img_pose, state_out);

                // ANCHOR -  remove point using PnP.
                if (op_track.remove_outlier_using_ransac_pnp(img_pose) == 0) //使用pnp去除野点，算出位姿
                {
                        cout << ANSI_COLOR_RED_BOLD << "****** Remove_outlier_using_ransac_pnp error*****" << ANSI_COLOR_RESET << endl;
                }
                g_cost_time_logger.record(tim, "Ransac");
                tim.tic("Vio_f2f");
                bool res_esikf = true, res_photometric = true;
                wait_render_thread_finish();
                // std::cout << "更新前" << state_out.pos_end << endl;

                /*Eigen::Matrix<double, 3, 3> tt; // H
                tt << 1, 1, 1, 1, 1, 1, 1, 1, 1;
                cout << " tt " << tt << endl;*/

                res_esikf = vio_esikf(state_out, op_track);
                // cout << "cout_solution " << cout_solution << endl;
                // cout << "cout_solution1 " << cout_solution1 << endl;
                g_cost_time_logger.record(tim, "Vio_f2f");
                tim.tic("Vio_f2m");
                // std::cout << "更新后" << state_out.pos_end << endl;
                res_photometric = vio_photometric(state_out, op_track, img_pose);

                g_cost_time_logger.record(tim, "Vio_f2m");
                g_lio_state = state_out; // vio更新后的状态给g_lio_state

                // std::cout << "重力: " << g_lio_state.gravity << endl;
                //  g_lio_state.pos_end(2) = 0.1;
                //  cout << "偏置为：" << g_lio_state.bias_a << "  " << "z轴：" << g_lio_state.pos_end(2) << endl;
                //  print_dash_board();
                set_image_pose(img_pose, state_out); //用状态预测结果作为这帧图像的初始位姿

                if (1)
                {
                        tim.tic("Render");
                        // m_map_rgb_pts.render_pts_in_voxels(img_pose, m_last_added_rgb_pts_vec);
                        if (1) // Using multiple threads for rendering
                        {
                                m_map_rgb_pts.m_if_get_all_pts_in_boxes_using_mp = 0;
                                // m_map_rgb_pts.render_pts_in_voxels_mp(img_pose, &m_map_rgb_pts.m_rgb_pts_in_recent_visited_voxels,
                                // img_pose->m_timestamp);
                                /*申请一个线程，在该函数中并行地将点云地图的active点投影到图像上，
                                用对应的像素对地图点rgb的均值和方差用贝叶斯迭代进行更新*/
                                m_render_thread = std::make_shared<std::shared_future<void>>(m_thread_pool_ptr->commit_task(
                                    render_pts_in_voxels_mp, img_pose, &m_map_rgb_pts.m_voxels_recent_visited, img_pose->m_timestamp));
                        }
                        else
                        {
                                m_map_rgb_pts.m_if_get_all_pts_in_boxes_using_mp = 0;
                                // m_map_rgb_pts.render_pts_in_voxels( img_pose, m_map_rgb_pts.m_rgb_pts_in_recent_visited_voxels,
                                // img_pose->m_timestamp );
                        }
                        m_map_rgb_pts.m_last_updated_frame_idx = img_pose->m_frame_idx;
                        g_cost_time_logger.record(tim, "Render");

                        tim.tic("Mvs_record");
                        if (m_if_record_mvs)
                        {
                                // m_mvs_recorder.insert_image_and_pts( img_pose, m_map_rgb_pts.m_voxels_recent_visited );
                                m_mvs_recorder.insert_image_and_pts(img_pose, m_map_rgb_pts.m_pts_last_hitted);
                        }
                        g_cost_time_logger.record(tim, "Mvs_record");
                }
                // ANCHOR - render point cloud
                dump_lio_state_to_log(m_lio_state_fp);
                m_mutex_lio_process.unlock();
                // cout << "Solve image pose cost " << tim.toc("Solve_pose") << endl;

                /*更新m_img_for_projection的位姿和id，以触发refresh线程Global_map::service_refresh_pts_for_projection()
                将点云的active点投影到2D（方便后面新增跟踪点）*/
                m_map_rgb_pts.update_pose_for_projection(img_pose, -0.4);
                op_track.update_and_append_track_pts(img_pose, m_map_rgb_pts, m_track_windows_size / m_vio_scale_factor, 1000000);
                g_cost_time_logger.record(tim, "Frame");
                double frame_cost = tim.toc("Frame");
                g_image_vec.push_back(img_pose);
                frame_cost_time_vec.push_back(frame_cost);
                if (g_image_vec.size() > 10)
                {
                        g_image_vec.pop_front();
                        frame_cost_time_vec.pop_front();
                }
                tim.tic("Pub");
                double display_cost_time = std::accumulate(frame_cost_time_vec.begin(), frame_cost_time_vec.end(), 0.0) / frame_cost_time_vec.size();
                g_vio_frame_cost_time = display_cost_time;
                // publish_render_pts( m_pub_render_rgb_pts, m_map_rgb_pts );
                publish_camera_odom(img_pose, message_time);
                // publish_track_img( op_track.m_debug_track_img, display_cost_time );
                publish_track_img(img_pose->m_raw_img, display_cost_time);

                if (m_if_pub_raw_img)
                {
                        publish_raw_img(img_pose->m_raw_img); //放置原始图片
                }

                if (g_camera_lidar_queue.m_if_dump_log)
                {
                        g_cost_time_logger.flush();
                }
                // cout << "Publish cost time " << tim.toc("Pub") << endl;
        }
}
