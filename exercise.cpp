std::vector<cv::KeyPoint> good_left, good_right;
std::vector<size_t> good_ids_left, good_ids_right;

for (size_t i=0; i<pts_left_new.size(); i++)
{
	if (pts_left_new[i].pt.x < 0 || pts_left_new[i].pt.y < 0 || (int)pts_right_new[i].pt.x > img_left.cols || (int)pts_right_new[i].pt.y > img_left.rows) continue;

	bool found_right = false;
	size_t index_right = 0;
	for (size_t n=0; n<ids_last[cam_id_right].size(); n++)
	{
		if (ids_last[cam_id_left].at(i)==ids_last[cam_id_right].at(n))
		{
			found_right = true;
			index_right = n;
			break;
		}
	}

	if (mask_ll[i] && found_right && mask_rr[index_right])
	{
		if (pts_right_new.at(index_right).pt.x < 0 || pts_right_new.at(index_right).pt.y < 0 || (int)pts_right_new[i].pt.x > img_right.cols || (int)pts_right_new[i].pt.y > img_right.rows) continue;
		good_left.push_back(pts_left_new.at(i));
		good_right.push_back(pts_right_new.at(index_right));
		good_ids_left.push_back(ids_last[cam_id_left].at(i));
		good_ids_right.push_back(ids_last[cam_id_right].at(index_right));
	} else if(mask_ll[i]) 
	{
		good_left.push_back(pts_left_new.at(i));
		good_ids_left.push_back(ids_last[cam_id_left].at(i));
	}
}


for (size_t i=0; i < pts_right_new.size(); i++)
{
	if (pts_right_new[i].pt.x < 0 || pts_right_new[i].pt.y < 0 || (int)pts_right_new[i].pt.x > img_right.cols || (int)pts_right_new[i].pt.y > img_right.rows) continue;

	bool added_already = (std::find(good_ids_right.begin(),good_ids_right.end(),ids_last[cam_id_right].at(i))!=good_ids_right.end());
	if (mask_rr[i] && !added_already) {
		good_right.push_back(pts_right_new.at(i));
		good_ids_right.push_back(ids_last[cam_id_right].at(i));
	}
}



















