//20201120-13:15
创建一个State对象

state = new State(state_options);
它先执行了 void State::initialize_variables()函数。
	在这个函数里初始化了IMU对象，_imu是指针类型，你可以建立对应类型的指针查看初始化的变量值。例如：
	****************************************************************************
	cout<<endl;
	cout<<"#######"<<endl;
	Vec* ptrv=_imu->p();
	JPLQuat* ptrq=_imu->q();
	cout<<ptrq->value()<<endl;
	cout<<ptrv->value()<<endl;
	ptrv=_imu->v();
	cout<<ptrv->value()<<endl;
	ptrv=_imu->bg();
	cout<<ptrv->value()<<endl;
	ptrv=_imu->ba();
	cout<<ptrv->value()<<endl;
	cout<<"#######"<<endl;
	cout<<_variables.size()<<endl;
	cout<<current_id<<endl;
	cout<<"#######"<<endl;
	****************************************************************************
	你可以看下输出是否符合预期！
	查看_options.num_cameras的值！
	cout<<_options.num_cameras<<endl;

	我们再看协方差的初始化
	cout << _calib_dt_CAMtoIMU->id() << endl;
	结果是15

	cout << _options.num_cameras << endl;
	结果是2

	根据程序current_id值的变化可以得出：
	pvqbabg是15维，1个IMU和相机的时间差dt是1维，6个左相机的位姿占6维(_options.do_calib_camera_pose=true说明优化相机位姿!)，8个左相机
	的内参数(_options.do_calib_camera_intrinsics=true说明优化相机内参)占8维,6个右相机的位姿占6维，8个右相机的内参数占8维，总计44维。

	初始化后 _Cov是44*44维的矩阵。

	对象state包含的变量：
	****************************************************************************
	double _timestamp;
	StateOptions _options;
	IMU *_imu;
	std::map<double, PoseJPL*> _clones_IMU;                        初始长度为0
	std::unordered_map<size_t, Landmark*> _features_SLAM;          初始长度为0
	Vec *_calib_dt_CAMtoIMU;                                       初始值为0
	std::unordered_map<size_t, PoseJPL*> _calib_IMUtoCAM;          初始长度为2
	std::unordered_map<size_t, Vec*> _cam_intrinsics;              初始长度为2
	std::unordered_map<size_t, bool> _cam_intrinsics_model;        初始长度为2
	Eigen::MatrixXd _Cov;                                          大小为44*44
	std::vector<Type*> _variables;                                 初始大小为6
	****************************************************************************


创建外state对象后，会根据从launch文件中读取的参数为state的对应参数赋值。

接下来如果launch文件中注明是用光流法那么就要创建一个光流法的对象！

	trackFEATS = new TrackKLT(num_pts,state->options().max_aruco_features,fast_threshold,grid_x,grid_y,min_px_dist);
	其内部主要执行的是TrackBase(numfeats, numaruco)
		而TrackBase内部又执行的是
			database(new FeatureDatabase()), num_features(200), currid(0) {}
			基类下的属性有：
			*********************************************************************************************************
			FeatureDatabase *database; 

			std::map<size_t, bool> camera_fisheye;

			std::map<size_t, cv::Matx33d> camera_k_OPENCV;

			std::map<size_t, cv::Vec4d> camera_d_OPENCV;

			int num_features;

			std::vector<std::mutex> mtx_feeds;

			std::map<size_t, cv::Mat> img_last;

			std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last;

			std::unordered_map<size_t, std::vector<size_t>> ids_last;

			std::atomic<size_t> currid;
			*********************************************************************************************************
			
			所以首先执行 new FeatureDatabase();

			其次执行 void set_calibration(std::map<size_t,Eigen::VectorXd> camera_calib, std::map<size_t, bool> camera_fisheye, 
							bool correct_active=false) (会调用多次)
				给定摄像机的参数，这将告诉我们应该用什么来归一化点。这也将用修正后的归一化值更新特征数据库。通常只有在优化摄像机参数                   时才需要这样做，因此应该重新归一化。
				源程序中std::map<size_t,Eigen::VectorXd> camera_calib的大小为2。
				相机参数指：[fx,fy,cx,cy,d1,d2,d3,d4]; 
				首次需要构建相机矩阵：
				fx   0     cx
				0    fx    cy
				0    0     1
				该矩阵一共有两个，cam.first(程序中有两个0和1)代表它的序号，tempK存放它的值。我们统一放在camera_k_OPENCV里它是一个map类                 型的容器！
				相对应的畸变参数我们同样以键值对的形式存放在camera_d_OPENCV中！

				如果不是首次也要转变一下数据的格式以适应opencv。

				如果我们正在校准相机的固有特性，我们的归一化坐标将是陈旧的。
				这是因为我们将它们附加到数据库中，并以当前最佳猜测*在该时间步长*。
				因此在这里既然我们的标定发生了变化，那么就把我们所有的特征重新归一化吧
				在 if (correct_active) 成立的条件下执行！
				cout << features_idlookup.size() << endl;可以实时查看特征数！
				里面有三层循环：
				第一层：for (const auto& pair_feat : features_idlookup)
					即循环每一个特征！
				第二层：for (auto const& meas_pair : feat->timestamps)
		        	cout << feat->timestamps.size() << endl; 值为2表明有左右两个相机
					所以第二层循环的是每一个相机关于该特征的信息！
					size_t camid = meas_pair.first; 它的值为0或1表示有两个相机！
				第三层：for (size_t m=0; m<feat->uvs.at(camid).size(); m++)
					cout << feat->uvs.at(camid).size() << endl;我们发现它的值从1到11再变为1如此循环往复，
					因此这个循环其实循环的是每一个特征在左或者右相机的长度为10的滑窗内的所有信息。
					feat->uvs.at(camid)的长度其实反映了某一特征在滑窗这个历史空间中前后被几个相机观测到。
					我们在循环内完成去畸变和归一化操作！
				/*
				补充：
					Feature (特征类) 的属性：
						size_t featid; 每个特征都会分到一个独一无二的id但是未必连续！
						bool to_delete; 是否该被删除的标志位！
						std::unordered_map<size_t,std::vector<Eigen::VectorXf> uvs;在被跟踪的一个特征点的所有信息
						std::unordered_map<size_t,std::vector<Eigen::vectorXf> uvs_norm;在被跟踪的一个特征点归一化后的所有信息
						std::unordered_map<size_t,std::vector<double>> timestamps;在被跟踪的一个特征点的所有时间戳信息
						int anchor_cam_id = -1; 参照相机
						double anchor_clone_timestamp; 参照相机时间戳
						Eigen::Vector3d p_FinA;三角化特征点相对参考相机的位置
						Eigen::Vector3d p_FinG;三角化特征点相对全局坐标系的位置
				*/


接下来会初始化传播对象

propagator = new Propagator(imu_noises,gravity);

很简单执行：Propagator(NoiseManager noises, Eigen::Vector3d gravity) : _noises(noises), _gravity(gravity);

/*
	补充：
		Propagator类下的公有属性：
		struct IMUDATA {
			double timestamp;
			Eigen::Matrix<double,3,1> wm;
			Eigen::Matrix<double,3,1> am;
		};

		struct NoiseManager {
			double sigma_w;
			double sigma_w_2;
			double sigma_wb;
			double sigma_wb_2;
			double sigma_a;
			double sigma_a_2;
			double sigma_ab;
			double sigma_ab_2;
		};

		Propagator的protected类型

		double last_prop_time_offset = -INFINITY;
		NoiseManager _noises;
		std::vector<IMUDATA> imu_data;
		Eigen::Matrix<double, 3, 1> _gravity;
*/

接下来建立初始化对象
initializer = new InertialInitializer(gravity, init_window_time, init_imu_thresh);

接下来创建UpdaterMSCKF对象
updaterMSCKF = new UpdaterMSCKF(msckf_options, featinit_options);

msckf_options的类型是UpdateOptions它的定义在文件UpdaterOptions.h中

struct UpdaterOptions {
	int chi2_multipler = 5;
	double sigma_pix = 1;
	double sigma_pix_sq = 1;
};

featinit_options的类型是FeatureInitializerOptions它的定义在文件FeatureInitializerOptions.h中

struct FeatureInitializerOptions {
	int max_runs = 20;
	double init_lamda = 1e-3;
	double max_lamda = 1e10;
	double min_dx = 1e-6;
	double min_dcost = 1e-6;
	double lam_mult = 10;
	double min_dist = 0.25;
	double max_dist = 40;
	double max_baseline = 40;
	double max_cond_number = 1000;
}

接下来创建UpdaterSLAM对象
updaterSLAM = new UpdaterSLAM(slam_options, aruco_options, featinit_options);

至此 sys = new VioManager(nh); 执行结束！
---------------------------------------------------------------------------------------------------------------------------------

接下来回到run_serial_msckf.cpp中执行：
viz = new RosVisualizer(nh, sys);

首先执行初始化函数:
RosVisualizer(ros::NodeHandle &nh, VioManager* app, Simulator* sim=nullptr);
建立发布者！

下面建立相机话题
std::string topic_imu;
std::string topic_camera0;
std::string topic_camera1;
nh.param<std::string>("topic_imu", topic_imu, "/imu0");
nh.param<std::string>("topic_camera0", topic_camera0, "/cam0/image_raw");
nh.param<std::string>("topic_camera1", topic_camera1, "/cam1/image_raw");

下面获取真实路径！
std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
if (nh.hasParam("path_gt"))
{
	std::string path_to_gt;
	nh.param<std::string>("path_gt", path_to_gt, "");
	DatasetReader::load_gt_file(path_to_gt, gt_states);
	/*
	补充：
		下面重点介绍 DatasetReader::load_gt_file函数！
		static void load_gt_file(std::string path, std::map<double,Eigen::Matrix<double,17,1>>& gt_states) {
			// 首先清理掉map类型的gt_states里的数据！
			gt_states.clear();

			std::ifstream file;
			std::string line;
			file.open(path);

			// 跳过第一行的表头！
			std::getline(file, line);

			// 循环读取每一行
			while (std::getline(file, line) && ros::ok()) {
				int i = 0;
				std::istringstream s(line);
				std::string field;
				Eigen::Matrix<double, 17, 1> temp;
				// 下面开始循环每一行的内容！
				while (getline(s, field, ',')) {
					if (i > 16)
					{
						std::exit(EXIT_FAILURE);
					}
					// 以浮点数的形式保存
					temp(i, 0) = std::atof(field.c_str());
					i++;
				}
				gt_states.insert({1e-9 * temp(0, 0), temp});
			}
			file.close();
		}
	*/
}


下面读取bag数据！

rosbag::Bag bag;
bag.open(path_to_bag, rosbag::bagmode::Read);

/*
	补充：
		rosbag的命令
			rosbag对软件包来操作，一个包是ROS用于存储ROS消息数据的文件格式，rosbag命令可以记录，回放和操作包。指令列表如下：
			**************************************************************************************************************
			check:      确定一个包是否可以在当前系统中进行，或者是否可以迁移。 
			decompress: 压缩一个或多个包文件。
			filter:     解压一个或多个包文件。
			fix:        在包文件中修复消息，以便在当前系统中播放。
			help:       获取相关命令指示帮助信息。
			info:       总结一个或多个包文件的内容。
			play:       以一种时间同步的方式回放一个或多个包文件的内容。
			record:     用指定主题的内容记录一个包文件。
			reindex:    重新索引一个或多个包文件。
			**************************************************************************************************************
*/


下面进入读取量测数据的大循环！

	首先是处理IMU量测！
		sys->feed_measurement_imu(timem, wm, am);
	其次是处理相机量测!
		img0 = cv_ptr->image.clone();
		time = cv_ptr->header.stamp.toSec();

		img0_buffer,img1_buffer是cv::Mat类型的图片！
	接下来执行if (max_cameras==2 && has_left && has_right)里的内容！
	而且第一次的时候要执行 sys->initialize_with_gt(imustate);

	从第二次开始就不用执行 sys->initialize_with_gt(imustate)了；
	而是执行 sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1)
	这开始进入核心了！

	开始执行：
	void TrackDescriptor::feed_stereo(double timestamp, cv::Mat &img_leftin, cv::Mat &img_rightin, size_t cam_id_left, size_t cam_id_right);
	重要变量：std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last;

	如果是循环的第一次也就是pts_last里没有数据那么要执行：
	void TrackKLT::perform_detection_stereo(const std::vector<cv::Mat> &img0pyr, const std::vector<cv::Mat> &img1pyr, std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1, std::vector<size_t> &ids0, std::vector<size_t> &ids1);
		在这个函数中首先判断相邻点是否挨得太近如果距离小于分辨率则只保留一个其余舍去，当然前提是得有点的数据，如若没有就不会执行循环！
		比如系统刚启动第一次读入数据是pts0等变量就没有数据自然不会进入该循环！

		int num_featsneeded = num_features - (int)pts0.size();
		这是在计算我们除去已经保留的特征还需要再提取多少特征！源程序中规定特征数为200，第一次pts0.size() = 0,所以num_featsneeded = 200.

		接下来就要提取特征了！执行：
		static void perform_griding(const cv::Mat &img, std::vector<cv::KeyPoint> &pts, int num_features, int grid_x, int grid_y, int threshold, bool nonmaxSuppression);
		核心是Fast特征提取!
		img.cols = 752;
		img.rows = 480;
		grid_x = 5;
		grid_y = 3;
		size_x = 150;
		size_y = 160;
		num_features_grid = 14, 每一个小区域大概提取14个点！
		ct_cols = grid_x = 5;
		ct_rows = grid_y = 3;
		std::vector<std::vector<cv::KeyPoint>> collection(ct_cols*ct_rows);一共存放15个部分！
		parallel_for_(cv::Range(0, ct_cols*ct_rows), [&](const cv::Range& range) { //15个小块是并行执行的。
			for (int r = range.start; r < range.end; r++)
			{
				int x = r%ct_cols*size_x;
				int y = r/ct_cols*size_y;

				if (x + size_x > img.cols || y + size_y > img.rows) continue;

				cv::Rect img_roi = cv::Rect(x, y, size_x, size_y);

				std::vector<cv::KeyPoint> pts_new;
				cv::FAST(img(img_roi), pts_new, threshold, nonmaxSuppression);

				std::sort(pts_new.begin(), pts_new.end(), Grider_FAST::compare_response);

				for (size_t i = 0; i < (size_t) num_features_grid && i < pts_new.size(); i++) {
					cv::KeyPoint pt_cor = pts_new.at(i);
					pt_cor.pt.x += x;
					pt_cor.pt.y += y;
					collection.at(r).push_back(pt_cor);
				}
			}
		});

	
		std::vector<cv::KeyPoint> kpts1_new; std::vector<cv::Point2f> pts1_new; 存放新提取的特征点！

		接下来使用光流法跟踪！
		cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0_new, pts1_new, mask, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);
		光流法输入参数：
		/*
		**********************************************************************************************************************
		img0pyr                       第一幅8位输入图像或由buildOpticalFlowPyramid()构造的金字塔
		img1pyr                       第二幅与img0pyr大小和类型相同的输入图像或金字塔
		pts0_new                      光流法需要找到的二维点的数组。点坐标必须是单精度浮点数！
		pts1_new                      包含输入特征在第二幅图像中计算出的新位置的二维点(单精度浮点坐标)的输出数组，当使用OPTFLOW_USE_INITIAL_FLOW标志时，pts1_new必须与pts0_new大小相同。
		mask                          状态向量，如果找到了对应特征的流，则将向量的每个元素设置为1否则为0。
		error                         误差输出向量，向量的每个元素被设置为对应特征的误差
		win_size                      每级金字塔的搜索窗口大小。
		pyr_levels                    基于最大金字塔层数，如果设置为0，则不使用金字塔(单级)，如果设置为1，则使用两个级别等等
		term_crit                     指定迭代搜索算法的终止准则
		cv::OPTFLOW_USE_INITIAL_FLOW  使用初始估计
		*********************************************************************************************************************
		*/
		最后把信息存在pts0,pts1包含新的和旧的特征点!(匹配要使用)

		接下来要执行：
		std::vector<uchar> mask_ll, mask_rr;
		std::vector<cv::KeyPoint> pts_left_new = pts_last[cam_id_left];
		std::vector<cv::KeyPoint> pts_right_new = pts_last[cam_id_right];
		void TrackKLT::perform_matching(const std::vector<cv::Mat>& img0pyr, const std::vector<cv::Mat>& img1pyr, std::vector<cv::KeyPoint>& kpts0, std::vector<cv::KeyPoint>& kpts1, size_t id0, size_t id1, std::vector<uchar>& mask_out)
		这里是进行左右相机的分别跟踪！

		接下来执行：
		std::vector<uchar> mask_lr;
		void TrackKLT::perform_matching(const std::vector<cv::Mat>& img0pyr, const std::vector<cv::Mat>& img1pyr, std::vector<cv::KeyPoint>& kpts0, std::vector<cv::KeyPoint>& kpts1, size_t id0, size_t id1, std::vector<uchar>& mask_out)
		这里是当前左右相机互跟踪！

		保存好的特征点，要求不能越界，同时mask_ll,mask_rr,mask_lr同时为真！
		good_left.size(),good_right.size()大约有70-140个。
		存放至database中：
		**************************************************************************************
		函数原型为：
		void update_feature(size_t id, double timestamp, size_t cam_id, float u, float v, float u_n, float v_n);
		**************************************************************************************
		database->update_feature(good_ids_left.at(i), timestamp, cam_id_left, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x,npt_l.y);
		database->update_feature(good_ids_left.at(i), timestamp, cam_id_right, good_right.at(i).pt.x, good_right.at(i).pt.y, npt_r.x, npt_r.y);
		**************************************************************************************
		重要变量：std::unordered_map<size_t, Feature *> features_idlookup;
		更新数据的程序！
		void update_feature(size_t id, double timestamp, size_t cam_id, float u, float v, float u_n, float v_n)
		{
			std::unique_lock<std::mutex> lck(mtx);
			if (features_idlookup.find(id) != features_idlookup.end()) {
				Feature *feat = features_idlookup[id];
				feat->uvs[cam_id].emplace_back(Eigen::Vector2f(u, v));
				feat->uvs_norm[cam_id].emplace_back(Eigen::Vector2f(u_n, v_n));
				feat->timestamps[cam_id].emplace_back(timestamp);
				return;
			}
        }	
		id号有随机性但是每个特征独一无二！！！！！！！
		变量features_idlookup的大小一般在60左右！
		feat->uvs
		feat->uvs_norm
		feat->timestamps
		三者的大小在1~12不等，上限取决于滑窗的长度！！！
		**************************************************************************************

		接下来如果 is_initialized_vio == false, 要执行：is_initialized_vio = try_to_initialize();
		/*
		______________________________________________________________________________________
		bool VioManager::try_to_initialize() {
			double time0;
			Eigen::Matrix<double, 4, 1> q_GtoI0;
			Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;
            // 用imu的数据进行初始化
			bool success = initialize_with_imu(time0, q_GtoI0, b_w0, v_I0inG, b_a0, p_I0inG);

			if (!success) {
				return false;
			}

			Eigen::Matrix<double, 16, 1> imu_val;
			imu_val.block(0,0,4,1) = q_GtoI0;
			imu_val.block(4,0,3,1) << 0, 0, 0;
			imu_val.block(7,0,3,1) = v_I0inG;
			imu_val.block(10,0,3,1) = b_w0;
			imu_val.block(13,0,3,1) = b_a0;
			state->imu()->set_value(imu_val);
			state->set_timestamp(time0);

			return true;
		}

		bool InertialInitializer::initialize_with_imu(double &time0, Eigen::Matrix<double,4,1> &q_GtoI0, Eigen::Matrix<double,3,1> &b_w0, Eigen::Matrix<double,3,1> &v_I0inG, Eigen::Matrix<double,3,1> &b_a0, Eigen::Matrix<double,3,1> &p_I0inG) {
			if (imu_data.empty()) {
				return false;   // imu_data为空则无法继续执行，只能返回！
			}

			double newesttime = imu_data.at(imu_data.size()-1).timestamp; // 获取最近的一个imu数据的时间戳！

			std::vector<IMUDATA> window_newest, window_secondnew;
			for (IMUDATA data : imu_data) {
				if (data.timestamp > newesttime-1*_window_length && data.timestamp <= newesttime-0*_window_length) {
					window_newest.push_back(data); // 存放最近的一个或多个imu数据!
				}
				if (data.timestamp > newesttime-2*_window_length && data.timestamp <= newesttime-1*_window_length) {
					window_secondnew.push_back(data); // 存放较近的一个或多个imu数据！
				}
			}

			if (window_newest.empty() || window_secondnew.empty()) {
				return false; // 如果都为空那么没有数据就无法继续执行！
			}

			Eigen::Matrix<double,3,1> a_avg = Eigen::Matrix<double,3,1>::Zero();//计算一个平均加速度漂移！
			for (IMUDATA data : window_newest) {
				a_avg += data.am;
			}
			a_avg /= (int)window_newest.size();// 平均加速度！
			double a_var = 0; // 加速度漂移方差!
			for (IMUDATA data : window_newest) {
				a_var += (data.am - a_avg).dot(data.am - a_avg);
			}
			a_var = std::sqrt(a_var/((int)window_newest.size()-1));

			if (a_var < _imu_excite_threshold) {
				return false;
			}

			Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
			Eigen::Vector3d angsum = Eigen::Vector3d::Zero();
			for (size_t i = 0; i < window_secondnew.size(); i++) {
				linsum += window_secondnew.at(i).am;
				angsum += window_secondnew.at(i).wm;
			}

			// 计算线加速和角速度的均值！
			Eigen::Vector3d linavg = Eigen::Vector3d::Zero();
			Eigen::Vector3d angavg = Eigen::Vector3d::Zero();
			linavg = linsum/(int)window_secondnew.size();
			angavg = angsum/(int)window_secondnew.size();

			Eigen::Vector3d z_axis = linavg/linavg.norm(); // 单位化
			Eigen::Vector3d e_1(1,0,0);
			// 创建 x_axis
			Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose()*e_1;
			x_axis = x_axis / x_axis.norm();

			Eigen::Matrix<double,3,1> y_axis = skew_x(z_axis)*x_axis;

			Eigen::Matrix<double,3,3> Ro;   // 用imu数据估计的初始位姿！
			Ro.block(0,0,3,1) = x_axis;
			Ro.block(0,1,3,1) = y_axis;
			Ro.block(0,2,3,1) = z_axis;

			Eigen::Matrix<double,4,1> q_GtoI = rot_2_quat(Ro); // 旋转矩阵转四元数!

			Eigen::Matrix<double,3,1> bg = angavg;
			Eigen::Matrix<double,3,1> ba = linavg - quat_2_Rot(q_GtoI) * _gravity;

			time0 = window_secondnew.at(window_secondnew.size()-1).timestamp;
			q_GtoI0 = q_GtoI;
			b_w0 = bg;
			v_I0inG = Eigen::Matrix<double,3,1>::Zero();
			b_a0 = ba;
			p_I0inG = Eigen::Matrix<double,3,1>::Zero();

			return true;

		}
		______________________________________________________________________________________
		*/
		


	接下来执行：void VioManager::do_feature_propagate_update(double timestamp);

		首先执行：void Propagator::propagate_and_clone(State* state, double timestamp)
		完成IMU状态预测和误差状态转移矩阵计算！
		**********************************************************************************************************************
		IMU预积分propagate_and_clone
		             |
					 |
					\|/                                          
		Step 1 IMU 数据预处理-------------------------------------> 调用interpolate_data进行IMU数据同步插值
		select_imu_readings                                      
		             |
					 |
					\|/                                         
		Step 2 IMU 预积分-----------------------------------------> 调用predict_mean_rk4/predict_mean_discrete进行预积分计算
		predict_and_compute                                     
		             |
					 |
					\|/
		Step 3 IMU 状态协方差
		预测并且累积Qd_summed
		             |
					 |
					\|/
		Step 4 更新MSCKF系统状态的协方差矩阵Cov
		             |
					 |
					\|/
		Step 5 相机状态增广
		augment_clone
		**********************************************************************************************************************
		**********************************************************************************************************************
		获取补偿后的时间！
		if (last_prop_time_offset == -INFINITY) {
			last_prop_time_offset = state->calib_dt_CAMtoIMU()->value()(0); // 其实一共就只有一个元素
		}

		double t_off_new = state->calib_dt_CAMtoIMU()->value()(0);

		double time0 = state->timestamp()+last_prop_time_offset; // 上次预积分结束时刻！
		double time1 = timestamp+t_off_new;                      // 相机当前时刻加上imu偏移的时间!
		**********************************************************************************************************************
		1 执行: std::vector<Propagator::IMUDATA> Propagator::select_imu_readings(const std::vector<IMUDATA>& imu_data, double time0, double time1);
		获取两帧图像间的imu数据！使用了插值，避免相机传感器时间戳对应无读数导致的积分误差！
		其中又执行: IMUDATA data = Propagator::interpolate_data(imu_data.at(i),imu_data.at(i+1),time0);

		2 执行: void Propagator::predict_and_compute(State *state, const IMUDATA data_minus, const IMUDATA data_plus, Eigen::Matrix<double,15,15> &F, Eigen::Matrix<double,15,15> &Qd);
		(predict_and_compute(state, prop_data.at(i), prop_data.at(i+1), F, Qdi);)

		imu读数先取出bias影响！
		*********************************************************************************************************************
		F.setZero();
		Qd.setZero();
		double dt = data_plus.timestamp-data_minus.timestamp;
		Eigen::Matrix<double,3,1> w_hat = data_minus.wm - state->imu()->bias_g();
		Eigen::matrix<double,3,1> a_hat = data_minus.am - state->imu()->bias_a();
		Eigen::matrix<double,3,1> w_hat2 = data_plus.wm - state->imu()->bias_g();
		Eigen::matrix<double,3,1> a_hat2 = data_plus.am - state->imu()->bias_a();
		*********************************************************************************************************************
		求解两帧imu数据间的预积分：
		predict_mean_discrete(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);
		_____________________________________________________________________________________________________________________
		state属性:                                               调用函数：
		double _timestamp;                                       double timestamp()
		StateOptions _options;                                   StateOptions& options()
		IMU *_imu;                                               IMU* imu()
		std::map<double, PoseJPL*> _clones_IMU;                  std::map<double, PoseJPL*> get_clones()
		std::unordered_map<size_t, Landmark*> _features_SLAM;    std::unordered_map<size_t, Landmark*> &features_SLAM()
		Vec *_calib_dt_CAMtoIMU;                                 Vec* calib_dt_CAMtoIMU()
		std::unordered_map<size_t, PoseJPL*> _calib_IMUtoCAM;    std::unordered_map<size_t, PoseJPL*> get_calib_IMUtoCAMs()
		std::unordered_map<size_t, Vec*> _cam_intrinsics;        Vec* get_intrinsics_CAM(size_t id)
		std::unordered_map<size_t, bool> _cam_intrinsics_model;  bool& get_model_CAM(size_t id)
		Eigen::MatrixXd _Cov;                                    Eigen::MatrixXd &Cov()
		std::vector<Type*> _variables;

		IMU属性:
		PoseJPL *_pose;                                          Eigen::Matrix<double, 3, 3> Rot() const
		Vec *_v;                                                 Eigen::Matrix<double, 3, 3> Rot_fej() const
		Vec *_bg;                                                Eigen::Matrix<double, 4, 1> quat() const
		Vec *_ba;                                                Eigen::Matrix<double, 4, 1> quat_fej() const
		                                                         Eigen::Matrix<double, 3, 1> pos() const
																 Eigen::Matrix<double, 3, 1> pos_fej() const
																 Eigen::Matrix<double, 3, 1> vel() const
																 Eigen::Matrix<double, 3, 1> vel_fej() const
																 Eigen::Matrix<double, 3, 1> bias_g() const
																 Eigen::Matrix<double, 3, 1> bias_g_fej() const
																 Eigen::Matrix<double, 3, 1> bias_a() const
																 Eigen::Matrix<double, 3, 1> bias_a_fej() const
																 PoseJPL *pose()
		                                                         JPLQuat *q()
		                                                         Vec *p()
		                                                         Vec *v()
		                                                         Vec *bg()
		                                                         Vec *ba()
		____________________________________________________________________________________________________________________
		旋转的左扰动和右扰动：
			定义Global坐标系为固定坐标系，如导航中常用的东北天坐标系；local坐标系为绑定在运动体上的坐标系，是随动的。
			一般旋转扰动的定义是在机体local坐标系上添加一个小的旋转 rLL'，这样扰动的旋转角比较小，可以保证比较好的线性而且避免
			奇异性，即RLL' = exp([rLL']^)。
			当位姿表达在Global坐标系时，根据旋转的叠加方式，添加的是右扰动：
			RGL' = RGL*RLL' = RGL(I+[rLL']^)
			当位姿表达在Local坐标系时，根据旋转的叠加方式，添加的是左扰动：
			RL'G = RL'L * RLG = (I + [rL'L]^)RLG = (I-[rLL']^)RLG
			在slam论文四元数的表达方式中，使用JPL表达旋转一般是将位姿表达在local下，所以其扰动是左扰动，而采用Hamilton表达
			旋转一般将位姿表达在global下，所以其扰动一般是右扰动。
		————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
		—————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
		直接求解离散误差传播方程中的F和G：
		详情见：推导11-24-01，推导11-24-02，推导11-24-03；
		Eigen::Matrix<double,3,3> Rfej = state->imu()->Rot_fej();
		Eigen::Matrix<double,3,3> dR = quat_2_Rot(new_q)*Rfej.transpose();

		Eigen::Matrix<double,3,1> v_fej = state->imu()->vel_fej();
		Eigen::Matrix<double,3,1> p_fej = state->imu()->pos_fej();

		F.block(th_id, th_id, 3, 3) = dR;
		F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;

		F.block(bg_id, bg_id, 3, 3).setIdentity();
		F.block(v_id, th_id, 3, 3).noalias() = -skew_x(new_v-v_fej+_gravity*dt)*Rfej.transpose();

		F.block(v_id, v_id, 3, 3).setIdentity();
		F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
		F.block(ba_id, ba_id, 3, 3).setIdentity();
		F.block(p_id, th_id, 3, 3).noalias() = -skew_x(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt)*Rfej.transpose();

		F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
		F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
		F.block(p_id, p_id, 3, 3).setIdentity();

		G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-w_hat * dt) * dt;

		G.block(v_id, 3, 3, 3) = -Rfej.transpose() * dt;
		G.block(p_id, 3, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
		G.block(bg_id, 6, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
		G.block(ba_id, 9, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();

		非fej：

		Eigen::Matrix<double,3,3> R_Gtoi = state->imu()->Rot();

		F.block(th_id, th_id, 3, 3) = exp_so3(-w_hat * dt);
		F.block(th_id, bg_id, 3, 3).noalias() = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
		F.block(bg_id, bg_id, 3, 3).setIdentity();
		F.block(v_id, th_id, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
		F.block(v_id, v_id, 3, 3).setIdentity();
		F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
		F.block(ba_id, ba_id, 3, 3).setIdentity();
		F.block(p_id, th_id, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
		F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
		F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
		F.block(p_id, p_id, 3, 3).setIdentity();

		G.block(th_id, 0, 3, 3) = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
		G.block(v_id, 3, 3, 3) = -R_Gtoi.transpose() * dt;
		G.block(p_id, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
		G.block(bg_id, 6, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
		G.block(ba_id, 9, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();

		——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

		根据两个imu的时间间隔，求解ba和bg噪声协方差Qc;
		——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
		Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
		Qc.block(0,0,3,3) = _noises.sigma_w_2/dt*Eigen::Matrix<double,3,3>::Identity();
		Qc.block(3,3,3,3) = _noises.sigma_a_2/dt*Eigen::Matrix<double,3,3>::Identity();
		Qc.block(6,6,3,3) = _noises.sigma_wb_2/dt*Eigen::Matrix<double,3,3>::Identity();
		Qc.block(9,9,3,3) = _noises.sigma_ab_2/dt*Eigen::Matrix<double,3,3>::Identity();
		——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
		计算噪声协方差的转移
		----------------------------------------------------------------------------------------------------------------------
		Qd = G*Qc*G.transpose();
		Qd = 0.5*(Qd+Qd.transpose());
		----------------------------------------------------------------------------------------------------------------------
		仅用imu数据对状态进行更新
		----------------------------------------------------------------------------------------------------------------------
		Eigen::Matrix<double,16,1> imu_x = state->imu()->value();
		imu_x.block(0,0,4,1) = new_q;
		imu_x.block(4,0,3,1) = new_p;
		imu_x.block(7,0,3,1) = new_v;
		state->imu()->set_value(imu_x);
		state->imu()->set_fej(imu_x);
		----------------------------------------------------------------------------------------------------------------------


		3. imu状态协方差计算
		Phi_summed = F * Phi_summed;
		Qd_summed = F * Qd_summed * F.transpose() + Qdi;
		Qd_summed = 0.5*(Qd_summed+Qd_summed.transpose());
		dt_summed += prop_data.ar(i+1).timestamp-prop_data.at(i).timestamp;
		这里的Qd_summed 是好多公式里的Pk+1|k

		4. 更新系统协方差矩阵
		---------------------------------------------------------------------------------------------------------------------
		Eigen::Matrix<double,3,1> last_w = prop_data.at(prop_data.size()-2).wm - state->imu()->bias_g();

		auto &Cov = state->Cov();
		size_t imu_id = state->imu()->id();
		Cov.block(imu_id,0,15,state->n_vars()) = Phi_summed*Cov.block(imu_id,0,15,state->n_vars());
		Cov.block(0,imu_id,state->n_vars(),15) = Cov.block(0,imu_id,state->n_vars(),15)*Phi_summed.transpose();
		Cov.block(imu_id,imu_id,15,15) += Qd_summed;
		Cov = 0.5*(Cov+Cov.transpose());

		state->set_timestamp(timestamp);
		last_prop_time_offset = t_off_new;
		Cov对应Pk|k; Qd_summed对应PIIk|k; Phi_summed对应 给PICk|k乘的系数矩阵！
		---------------------------------------------------------------------------------------------------------------------

		5. 状态增广，考虑时间偏移对协方差校正！
		StateHelper::augment_clone(state, last_w);
		void StateHelper::augment_clone(State *state, Eigen::Matrix<double, 3, 1> last_w);
			里面首先执行 增广矩阵函数:把当前的imu预积分值作为相机的初始值增广到MSCKF协方差矩阵中：
			Type *posetemp = StateHelper::clone(state, imu->pose());
			Type* StateHelper::clone(State *state, Type *variable_to_clone);
			/*
			----------------------------------------------------------------------------------------------------------------
			int total_size = variable_to_clone->size();    total_size = 6 指代表示imu姿态的6个量！
			int old_size = (int) state->n_vars();          old_size = 116 = 44 + 72 44表示状态协方差的维度，72表示6*12,12个imu位姿的状态
			int new_loc = (int) state->n_vars();           new_loc = 116 同理！

			拷贝已有的协方差的值，相同的IMU的协方差拷贝到新增的相机状态处
			Type* type_check = state->variables(k)->check_if_same_variable(variable_to_clone);

			state->Cov().block(new_loc, new_loc, total_size, total_size) = state->Cov().block(old_loc, old_loc, total_size, total_size);
			state->Cov().block(0, new_loc, old_size, total_size) = state->Cov().block(0, old_loc, old_size, total_size);
			state->Cov().block(new_loc, 0, total_size, old_size) = state->Cov().block(old_loc, 0, total_size, old_size);

			把新的状态，等同于相机的姿态增加到state的变量中；
			另外见：推导11-25-01
			----------------------------------------------------------------------------------------------------------------
			*/
			插入imu的状态到state的变量_clones_IMU中。
			然后根据时间偏移量对IMU状态进行校正
			if (state->options().do_calib_camera_timeoffset) {
				Eigen::Matrix<double, 6, 1> dnc_dt = Eigen::MatrixXd::Zero(6, 1);
				dnc_dt.block(0, 0, 3, 1) = last_w;
				dnc_dt.block(3, 0, 3, 1) = imu->vel();

				Cov.block(0, pose->id(), Cov.rows(), 6) += Cov.block(0, state->calib_dt_CAMtoIMU()->id(), Cov.rows(), 1) * dnc_dt.transpose();
				Cov.block(pose->id(), 0, 6, Cov.rows()) += dnc_dt * Cov.block(state->calib_dt_CAMtoIMU()->id(), 0, 1, Cov.rows());
				Cov.block(pose->id(), pose->id(), 6, 6) += dnc_dt * Cov(state->calib_dt_CAMtoIMU()->id(),state->calib_dt_CAMtoIMU()->id()) * dnc_dt.transpose();
			}

	
		3. MSCKF features and KLT tracks that are SLAM features.
			获取过时的量测：
			std::vector<Feature*> feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->timestamp());
			获取要使用的特征：
			if ((int)state->n_clones() > state->options().max_clone_size)
				feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep());
				/*
				-------------------------------------------------------------------------------------------------------------
				State.h
				double margtimestep() {
					double time = INFINITY;
					for (std::pair<const double, PoseJPL *> &clone_imu : _clones_IMU) {
						if (clone_imu.first < time) {
							time = clone_imu.first;
						}
					}
					return time;
				}
				找出最早的时间，也就是我们要边缘化的时间！
				-------------------------------------------------------------------------------------------------------------

				-------------------------------------------------------------------------------------------------------------
				FeatureDatabase.h
				std::vector<Feature *> features_containing(double timestamp, bool remove=false);
				*************************************************************************************************************
				变量说明:
				std::unordered_map<size_t, Feature *> features_idlookup; size() 一般为两位数！
				std::unordered_map<size_t, std::vector<double>> timestamps; size()为2，因为是左右两个相机！
				timestamps.second对应一个vector<double> 它的长度在1~12，上限取决于滑窗长度！
				*************************************************************************************************************

				寻找跟踪超过一定时长的特征，这些可以作为SLAM特征！
				std::vector<Feature*> feats_maxtracks;
				auto it2 = feats_marg.begin();
				while (it2 != feats_marg.end()) {
					bool reached_max = false;
					for (const auto &cams : (*it2)->timestamp) {
						if ((int)cams.second.size() > state->options().max_clone_size) {
							reached_max = true;
							break;
						}
					}

					if (reached_max) {
						feats_maxtracks.push_back(*it2);
						it2 = feats_marg.erase(it2);
					} else {
						it2++;
					}
				}

				如果还有空间则加入slam特征！
				if (state->options().max_slam_features > 0 && timestamp-startup_time >= dt_statupdelay && (int)state->features_SLAM().size() < state->options().max_slam_features+curr_aruco_tags) {
					int amount_to_add = (state->options().max_slam_features+curr_aruco_tags)-(int)state->features_SLAM().size();
					int valid_amount = (amount_to_add > (int)feats_maxtracks.size()) ? (int)feats_maxtracks.size() : amount_to_add;

					if (valid_amount > 0) {
						feats_slam.insert(feats_slam.end(), feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
						feats_maxtracks.erase(feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
					}
				}
				-------------------------------------------------------------------------------------------------------------
				*/

				去除要边缘化的点
				StateHelper::marginalize_slam(state);(static void marginalize_slam(State* state)
				执行：StateHelper::marginalize(state,(*it0).second);(void StateHelper::marginalize(State *state, Type *marg))
				首先会确认状态中是否有slam的landmark；
				直接在协方差矩阵中去除slam特征点对应的协方差块！
				/*
					注：这里的Type *marg 对应 std::unordered_map<size_t, Landmark*> _features_SLAM;
				*/
				/*
				------------------------------------------------------------------------------------------------------------
				int marg_size = marg->size();
				int marg_id = marg->id();

				Eigen::MatrixXd Cov_new(state->n_vars() - marg_size, state->n_vars() - marg_size);

				int x2_size = (int)state->n_vars() - marg_id - marg_size;

				Cov_new.block(0, 0, marg_id, marg_id) = state->Cov().block(0, 0, marg_id, marg_id);

				Cov_new.block(0, marg_id,  marg_id, x2_size) = state->Cov().block(0, marg_id + marg_size, marg_id, x2_size);

				Cov_new.block(marg_id, 0, x2_size, marg_id) = Cov_new.block(0, marg_id, marg_id, x2_size).transpose();

				Cov_new.block(marg_id, marg_id, x2_size, x2_size) = state->Cov().block(marg_id + marg_size, marg_id + marg_size, x2_size, x2_size);
				state->Cov() = Cov_new;

				去除VIO中被边缘化的量并做重新调整！
				std::vector<Type *> remaining_variables;
				for (size_t i = 0; i < state->variables().size(); i++) {
					if (state->variables(i) != marg_id) {
						// 如果没有被边缘化
						if (state->variables(i)->id() > marg_id) {
							// 如果该变量的id在被边缘化的变量之后，就要统一前移
							state->variables(i)->set_local_id(state->variables(i)->id() - marg_size);
						}
						remaining_variables.push_back(state->variables(i));
					}
				}

				delete marg;

				state->variables() = remaining_variables;
				------------------------------------------------------------------------------------------------------------
				*/

				------------------------------------------------------------------------------------------------------------
				把本次新产生的特征中新的和旧的分离！
				std::vector<Feature*> feats_slam_DELAYED, feats_slam_UPDATE;
				for (size_t i=0; i<feats_slam.size(); i++) {
					if (state->features_SLAM().find(feats_slam.at(i)->featid) != state->features_SLAM().end()) {
						feats_slam_UPDATE.push_back(feats_slam.at(i));
					} else {
						feats_slam_DELAYED.push_back(feats_slam.at(i));
					}	
				}

				全部放到MSCKF特征里！
				std::vector<Feature*> featsup_MSCKF = feats_lost;
				featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
				featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());
				------------------------------------------------------------------------------------------------------------
				featsup_MSCKF大概二十多个点！

				更新：updaterMSCKF->update(state, featsup_MSCKF);
					0.获取滑窗内的时间戳！
					--------------------------------------------------------------------------------------------------------
					std::vector<double> clonetimes;
					for (const auto& clone_imu : state->get_clones()) {
						clonetimes.emplace_back(clone_imu.first);
					} 长度一般为12
					--------------------------------------------------------------------------------------------------------

					1.去掉不在滑窗时间范围内的特征,只关注滑窗内的特征点就足够了！
					--------------------------------------------------------------------------------------------------------
					从featsup_MSCKF里每次取一个特征然后执行：clean_old_measurements(clonetimes);
					/*
						void Feature::clean_old_measurements(std::vector<double> valid_times) {
							for (auto const &pair : timestamps) {
								
								assert(timestamps[pair.first].size() == uvs[pair.first].size());
								assert(timestamps[pair.first].size() == uvs_norm[pair.first].size());

								auto it1 = timestamps[pair.first].begin();
								auto it2 = uvs[pair.first].begin();
								auto it3 = uvs_norm[pair.first].begin();

								while (it1 != timestamps[pair.first].end()) {
									if (std::find(valid_times.begin(),valid_times.end(),*it1) == valid_times.end()) {
										it1 = timestamps[pair.first].erase(it1);
										it2 = uvs[pair.first].erase(it2);
										it3 = uvs_norm[pair.first].erase(it2);
									} else {
										++it1;
										++it2;
										++it3;
									}
								}
							}
						}
					*/
					--------------------------------------------------------------------------------------------------------
					2.对左右相机，根据滑窗内的所有IMU位姿，推算每对相机的位姿！
					/*
					--------------------------------------------------------------------------------------------------------
					std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
					for (const auto &clone_calib : state->get_calib_IMUtoCAMs()) { // 两次循环分别使用左右相机的内参
						
						std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
						for (const auto &clone_imu : state->get_clones()) {
							// 循环每一个时刻的状态!
							// 得到当前相机位姿!
							Eigen::Matrix<double,3,3> R_GtoCi = clone_calib.second->Rot()*clone_imu.second->Rot();
							Eigen::Matrix<double,3,1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose()*clone_calib.second->pos();
							clones_cami.insert({clone_imu.first,FeatureInitializer::ClonePose(R_GtoCi,p_CioinG)});
						}

						clones_cam.insert({clone_calib.first,clone_cami});
					}
					clone_calib是从imu系到相机Ci系的转移矩阵，clone_imu.second->Rot()指代q_GtoI从全局到imu的转移矩阵！
					R(GtoCi) = R(itoCi) * R(Gtoi);
					P(itoG) = P(CitoG) + R(CitoG)*P(itoCi);
					所以:P(CitoG) = P(itoG) - R(CitoG)*P(itoCi);
					--------------------------------------------------------------------------------------------------------
					*/

					3.三角化特征点的3D坐标，(已知滑窗内所有的相机位姿，和特征点的2D匹配点坐标)先三角化初始值再BA优化！
					--------------------------------------------------------------------------------------------------------
					auto it1 = feature_vec.begin();
					while (it1 != feature_vec.end()) {//每次取出一个特征
						bool success = initializer_feat->single_triangulation(*it1, clones_cam);
						// 三角化失败的话删除特征进入下一次循环！
						/*
							
						*/
					}
					--------------------------------------------------------------------------------------------------------
























































