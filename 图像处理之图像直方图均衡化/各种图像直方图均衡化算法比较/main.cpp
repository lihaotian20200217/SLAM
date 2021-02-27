#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

cv::Mat equalizeHist_realize(cv::Mat src)
{
	int width = src.cols;
	int height = src.rows;
	cv::Mat eH_img = src.clone();
	int tmp[256] = {0};
	float T[256] = {0.0};
	int total = width * height;
	//计算每个值对应的像素个数
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			int index = src.at<uchar>(i,j);
			tmp[index]++;
		}
	}
	//计算累计函数
	for (int i = 0; i < 256; i++)
	{
		if (i == 0)
		{
			T[i] = 1.0f * tmp[i] / total;
		}
		else
		{
			T[i] = T[i-1] + 1.0f * tmp[i] / total;
		}
	}
	//进行映射
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			int index = src.at<uchar>(i,j);
			eH_img.at<uchar>(i,j) = T[index] * 255;
		}
	}
	return eH_img;
}


cv::Mat AHE(cv::Mat src, int _step = 8)
{
	cv::Mat AHE_img = src.clone();
	int block = _step;
	int width = src.cols;
	int height = src.rows;
	int width_block = width / block;   //小区域的宽
	int height_block = height / block; //小区域的长
	//存储各个局部直方图
	int tmp2[8*8][256] = {0};
	int T2[8*8][256] = {0.0};
	//分块
	int total = width_block * height_block;
	for (int i = 0; i < block; i++)
	{
		for (int j = 0; j < block; j++)
		{
			int start_x = i * width_block;
			int end_x = start_x + width_block;
			int start_y = j * height_block;
			int end_y = start_y + height_block;
			int num = i + block * j;
			//遍历小块计算直方图
			for (int ii = start_x; ii < end_x; ii++)
			{
				for (int jj = start_y; jj < end_y; jj++)
				{
					int index = src.at<uchar>(jj,ii);
					tmp2[num][index]++;
				}
			}
			//计算累计分布直方图
			for (int k = 0; k < 256; k++)
			{
				if (k == 0)
				{
					T2[num][k] = 1.0f * tmp2[num][k] / total;
				}
				else
				{
					T2[num][k] = T2[num][k-1] + 1.0f * tmp2[num][k] / total;
				}
			}
		}
	}
	//映射到新的图像
	for (int i = 0; i < block; i++)
	{
		for (int j = 0; j < block; j++)
		{
			int start_x = i * width_block;
			int end_x = start_x + width_block;
			int start_y = j * height_block;
			int end_y = start_y + height_block;
			int num = i + block * j;
			//遍历小块计算直方图
			for (int ii = start_x; ii < end_x; ii++)
			{
				for (int jj = start_y; jj < end_y; jj++)
				{
					int index = src.at<uchar>(jj,ii);
					AHE_img.at<uchar>(jj,ii) = T2[num][index] * 255;
				}
			}
		}
	}
	return AHE_img;
}

cv::Mat clhe(cv::Mat src, int _step = 8)
{
	int width = src.cols;
	int height = src.rows;
	cv::Mat CLHE_img = src.clone();
	int tmp[256] = {0};
	float T[256] = {0.0};
	int total = width * height;
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			int index = src.at<uchar>(i,j);
			tmp[index]++;
		}
	}

	int average = width * height / 255 / 64;
	int LIMIT = 4 * average;
	int steal = 0;
	for (int k = 0; k < 256; k++)
	{
		if (tmp[k] > LIMIT)
		{
			steal += tmp[k] - LIMIT;    //超出限制的先累加在steal中
			tmp[k] = LIMIT;             //限制tmp[k]的数量不能超过LIMIT
		}
	}
	int bonus = steal / 256;
	for (int k = 0; k < 256; k++)
	{
		tmp[k] += bonus;                //把多余的累加在steal中的值平均加在每个tmp[k]中
	}
	//
	for (int i = 0; i < 256; i++)
	{
		if (i == 0)
		{
			T[i] = 1.0f * tmp[i] / total;
		}
		else
		{
			T[i] = T[i-1] + 1.0f * tmp[i] / total;
		}
	}

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			int index = src.at<uchar>(i,j);
			CLHE_img.at<uchar>(i,j) = T[index] * 255;
		}
	}
	return CLHE_img;
}

cv::Mat claheWithoutInterpolation(cv::Mat src, int _step = 8)
{
	cv::Mat CLAHE_img = src.clone();
	int block = _step;
	int width = src.cols;
	int height = src.rows;
	int width_block = width / block;
	int height_block = height / block;

	int tmp2[8*8][256] = {0};
	float T2[8*8][256] = {0.0};
	int total = width_block * height_block;
	for (int i = 0; i < block; i++)
	{
		for (int j = 0; j < block; j++)
		{
			int start_x = i * width_block;
			int end_x = start_x + width_block;
			int start_y = j * height_block;
			int end_y = start_y + height_block;
			int num = i + block * j;
			//遍历小块，计算直方图
			for (int ii = start_x; ii < end_x; ii++)
			{
				for (int jj = start_y; jj < end_y; jj++)
				{
					int index = src.at<uchar>(jj,ii);
					tmp2[num][index]++;
				}
			}
			//针对每一个小块
			int average = width_block * height_block / 255;
			int LIMIT = 4 * average;
			int steal = 0;
			for (int k = 0; k < 256; k++)
			{
				if (tmp2[num][k] > LIMIT)
				{
					steal += tmp2[num][k] - LIMIT;
					tmp2[num][k] = LIMIT;
				}
			}
			int bonus = steal / 256;
			for (int k = 0; k < 256; k++)
			{
				tmp2[num][k] += bonus;
			}
			for (int k = 0; k < 256; k++)
			{
				if (k == 0)
				{
					T2[num][k] = 1.0f * tmp2[num][k] / total;
				}
				else
				{
					T2[num][k] = T2[num][k-1] + 1.0f * tmp2[num][k] / total;
				}
			}
		}
	}
	for (int i = 0; i < block; i++)
	{
		for (int j = 0; j < block; j++)
		{
			int start_x = i * width_block;
			int end_x = start_x + width_block;
			int start_y = j * height_block;
			int end_y = start_y + height_block;
			int num = i + block * j;

			for (int ii = start_x; ii < end_x; ii++)
			{
				for (int jj = start_y; jj < end_y; jj++)
				{
					int index = src.at<uchar>(jj,ii);
					CLAHE_img.at<uchar>(jj,ii) = T2[num][index] * 255;
				}
			}
		}
	}
	return CLAHE_img;
}

cv::Mat clahe(cv::Mat src, int _step = 8)
{
	cv::Mat CLAHE_img = src.clone();
	int block = _step;
	int width = src.cols;
	int height = src.rows;
	int width_block = width / block;
	int height_block = height / block;
	//存储各个直方图
	int tmp2[8*8][256] = {0};
	float T2[8*8][256] = {0.0};
	//分块
	int total = width_block * height_block;
	for (int i = 0; i < block; i++)
	{
		for (int j = 0; j < block; j++)
		{
			int start_x = i * width_block;
			int end_x = start_x + width_blocks;
			int start_y = j * height_block;
			int end_y = start_y + height_block;
			int num = i + block * j;
			//遍历小块，计算直方图
			for (int ii = start_x; ii < end_x; ii++)
			{
				for (int jj = start_y; jj < end_y; jj++)
				{
					int index = src.at<uchar>(jj,ii);
					tmp2[num][index]++;
				}
			}
			int average = width_block * height_block / 255;
			int LIMIT = 40 * average;
			int steal = 0;
			for (int k = 0; k < 256; k++)
			{
				if (tmp2[num][k] > LIMIT)
				{
					steal += tmp2[num][k] - LIMIT;
					tmp2[num][k] = LIMIT;
				}
			}
			int bonus = steal / 256;

			for (int k = 0; k < 256; k++)
			{
				tmp2[num][k] += bonus;
			}
			for (int k = 0; k < 256; k++)
			{
				if (k == 0)
				{
					T2[num][k] = 1.0f * tmp2[num][k] / total;
				}
				else
				{
					T2[num][k] = T2[num][k-1] + 1.0f * tmp2[num][k] / total;
				}
			}
		}
	}
	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < height; j++)
		{
			if (i <= width_block/2 && j <= height_block/2)
			{
				int num = 0;
				CLAHE_img.at<uchar>(j,i) = (int)(T2[num][CLAHE_img.at<uchar>(j,i)] * 255);
			}
			else if (i <= width_block/2 && j >= ((block-1)*height_block + height_block/2))
			{
				int num = block * (block - 1);
				CLAHE_img.at<uchar>(j,i) = (int)(T2[num][CLAHE_img.at<uchar>(j,i)] * 255);
			}
			else if (i >= ((block-1) * width_block + width_block / 2) && j <= height_block / 2)
			{
				int num = block - 1;
				CLAHE_img.at<uchar>(j,i) = (int)(T2[num][CLAHE_img.at<uchar>(j,i)] * 255);
			}
			else if (i >= ((block - 1) * width_block + width_block / 2) && j >= ((block - 1) * height_block + height_block / 2))
			{
				int num = block * block - 1;
				CLAHE_img.at<uchar>(j,i) = (int)(T2[num][CLAHE_img.at<uchar>(j,i)] * 255);
			}
			else if (i <= width_block / 2)
			{
				int num_i = 0;
				int num_j = (j - height_block / 2) / height_block;
				int num1 = num_j * block + num_i;
				int num2 = num1 + block;
				float p = (j - (num_j * height_block + height_block / 2)) / (1.0f * height_block);
				float q = 1 - p;
				CLAHE_img.at<uchar>(j,i) = (int)((q * T2[num][CLAHE_img.at<uchar>(j,i)] + p * T2[num2][CLAHE_img.at<uchar>(j,i)]) * 255);
			}
			else if (i >= ((block - 1) * width_block + width_block / 2))
			{
				int num_i = block - 1;
				int num_j = (j - height_block / 2) / height_block;
				int num1 = num_j * block + num_i;
				int num2 = num1 + block;
				float p = (j - (num_j * height_block + height_block / 2)) / (1.0f * height_block);
				float q = 1 - p;
				CLAHE_img.at<uchar>(j,i) = (int)((q * T2[num1][CLAHE_img.at<uchar>(j,i)] + p * T2[num2][CLAHE_img.at<uchar>(j,i)]) * 255);
			}
			else if (j <= height_block / 2)
			{
				int num_i = (i - width_block / 2) / width_block;
				int num_j = 0;
				int num1 - num_j * block + num_i;
				int num2 = num1 + 1;
				float p = (i - (num_i * width_block + width_block / 2)) / (1.0f * width_block);
				float q = 1 - p;
				CLAHE_img.at<uchar>(j,i) = (int)((q * T2[num1][CLAHE_img.at<uchar>] + p * T2[num2][CLAHE_img.at<uchar>(j,i)]) * 255);
			}
			else if (j >= ((block - 1) * height_block + height_block / 2))
			{
				int num_i = (i - width_block / 2) / width_block;
				int num_j = block - 1;
				int num1 = num_j * block + num_1;
				int num2 = num1 + 1;
				float p = (i - (num_i * width_block + width_block / 2)) / (1.0f * width_block);
				float q = 1 - p;
				CLAHE_img.at<uchar>(j,i) = (int)((q * T2[num1][CLAHE_img.at<uchar>(j,i)] + p * T2[num2][CLAHE_img.at<uchar>(j,i)]) * 255);
			}
			else
			{
				int num_i = (i - width_block / 2) / width_block;
				int num_j = (j - height_block / 2) / height_block;
				int num1 = num_j * block + num_i;
				int num2 = num1 + 1;
				int num3 = num1 + block;
				inr num4 = num2 + block;
				float u = (i - (num_i * width_block + width_block / 2)) / (1.0f * width_block);
				float v = (j - (num_j * height_block + height_block / 2)) / (1.0f * height_block);
				CLAHE_img.at<uchar>(j,i) = (int)((u * v * T2[num4][CLAHE_img.at<uchar>(j,i)]
										+(1-v) * (1-u) * T2[num1][CLAHE_img.at<uchar>(j,i)]
										+u * (1 - v) * T2[num2][CLAHE_img.at<uchar>(j,i)]
										+v * (1 - u) * T2[num3][CLAHE_img.at<uchar>(j,i)]) * 255);
			}
			CLAHE_img.at<uchar>(j,i) = CLAHE_img.at<uchar>(j,i) + (CLAHE_img.at<uchar>(j,i) << 8) + (CLAHE_img.at<uchar>(j,i) << 16);
		}
	}
	return CLAHE_img;
}

int main()
{
	Mat src = imread("your pictures!",0);
	
	Mat ht_opencv;
	Mat ht_;
	Mat ahe_;
	Mat clhe;
	Mat clahe_without_interpolation;
	Mat clahe_opencv;
	Mat clahe_;

	cv::equalizaHist(src,ht_opencv);
	ht_ = equalizaHist_realize(src);
	ahe_ = AHE(src);
	clhe = clhe(src);
	clahe_without_interpolation = claheWithoutInterpolation(src);
	Ptr<cv::CLAHE> clahe = createCLAHE();
	clahe->apply(src,clahe_opencv);
	clahe_ = clahe(src);

	cv::imshow("src",src);
	cv::imshow("ht_opencv",ht_opencv);
	cv::imshow("ht_",ht_);
	cv::imshow("ahe_",ahe_);
	cv::imshow("clhe",clhe);
	cv::imshow("clahe_without_interpolation",clahe_without_interpolation);
	cv::imshow("clahe_opencv",clahe_opencv);
	cv::imshow("clahe_",clahe_);

	waitKey();

	return 0;

}








































