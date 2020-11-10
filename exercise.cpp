void TrackKLT::perform_matching(const std::vector<cv::Mat>& img0pyr, const std::vector<cv::Mat>& img1pyr, std::vector<cv::KeyPoint>& kpts0, std::vector<cv::KeyPoint>& kpts1, size_t id0, size_t id1, std::vector<uchar>& mask_out) {
	
	assert(kpts0.size() == kpts1.size());

	if (kpts0.empty() || kpts1.empty()) return;

	std::vector<cv::Point2f> pts0, pts1;
	for (size_t i=0; i < kpts0.size(); i++)
	{
		pts0.push_back(kpts0.at(i).pt);
		pts1.push_back(kpts1.at(i).pt);
	}

	if (pts0.size() < 10)
	{
		for (size_t i=0; i < pts0.size(); i++)
		{
			mask_out.push_back((uchar)0);
		}
		return;
	}

	std::vector<uchar> mask_klt;
	std::vector<float> error;
	cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 15, 0.01);
	cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0, pts1, mask_klt, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);

	std::vector<cv::Point2f> pts0_n, pts1_n;
	for (size_t i=0; i < pts0.size(); i++)
	{
		pts0_n.push_back(undistort_point(pts0.at(i),id0));
		pts1_n.push_back(undistort_point(pts1.at(i),id1));
	}

	std::vector<uchar> mask_rsc;
	double max_focallength_img0 = std::max(camera_k_OPENCV.at(id0)(0,0),camera_k_OPENCV.at(id0)(1,1));
	double max_focallength_img1 = std::max(camera_k_OPENCV.at(id1)(0,0),camera_k_OPENCV.at(id1)(1,1));
	double max_focallength = std::max(max_focallength_img0,max_focallength_img1);
	cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, 1/max_focallength, 0.999, mask_rsc);

	for (size_t i=0; i < mask_klt.size(); i++)
	{
		auto mask = (uchar)((i < mask_klt.size() && mask_klt[i] && i < mask_rsc.size() && mask_rsc[i])? 1 : 0);
		mask_out.push_back(mask);
	}
	
	for (size_t i=0; i < pts0.size(); i++)
	{
		kpts0.at(i).pt = pts0.at(i);
		kpts1.at(i).pt = pts1.at(i);
	}

}
