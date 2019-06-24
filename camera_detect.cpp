#include "stdafx.h"
#include "camera_detect.h"
#include "E:\opencv\opencv\build\include\opencv2\opencv.hpp"
#include "E:\opencv\opencv\build\include\opencv2\core\core.hpp"
#include "E:\opencv\opencv\build\include\opencv2\highgui\highgui.hpp"
#include "E:\opencv\opencv\build\include\opencv2\imgproc\imgproc.hpp"
#include "E:\opencv\opencv\build\include\opencv2\imgproc\imgproc_c.h"
#include <windows.h>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;


const int imageWidth = 640;                             //摄像头的分辨率  
const int imageHeight = 480;
const int boardWidth = 11;                               //横向的角点数目  
const int boardHeight = 8;                              //纵向的角点数据  
const int boardCorner = boardWidth * boardHeight;       //总的角点数据  
const int frameNumber = 5;                             //相机标定时需要采用的图像帧数  
const int squareSize = 10;                              //标定板黑白格子的大小 单位mm  
const Size boardSize = Size(boardWidth, boardHeight);   //  

Mat intrinsic;                                          //相机内参数  
Mat distortion_coeff;                                   //相机畸变参数  
vector<Mat> rvecs;                                        //旋转向量  
vector<Mat> tvecs;                                        //平移向量  
vector<vector<Point2f>> corners;                        //各个图像找到的角点的集合 和objRealPoint 一一对应  
vector<vector<Point3f>> objRealPoint;                   //各副图像的角点的实际物理坐标集合  


vector<Point2f> corner;                                   //某一副图像找到的角点  

Mat rgbImage, grayImage;

/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	//  Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));  
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			//  imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);  
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

/*设置相机的初始参数 也可以不估计*/
void guessCameraParam(void)
{
	/*分配内存*/
	intrinsic.create(3, 3, CV_64FC1);
	distortion_coeff.create(5, 1, CV_64FC1);

	/*
	fx 0 cx
	0 fy cy
	0 0  1
	*/
	intrinsic.at<double>(0, 0) = 256.8093262;   //fx         
	intrinsic.at<double>(0, 2) = 160.2826538;   //cx  
	intrinsic.at<double>(1, 1) = 254.7511139;   //fy  
	intrinsic.at<double>(1, 2) = 127.6264572;   //cy  

	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	/*
	k1 k2 p1 p2 p3
	*/
	distortion_coeff.at<double>(0, 0) = -0.193740;  //k1  
	distortion_coeff.at<double>(1, 0) = -0.378588;  //k2  
	distortion_coeff.at<double>(2, 0) = 0.028980;   //p1  
	distortion_coeff.at<double>(3, 0) = 0.008136;   //p2  
	distortion_coeff.at<double>(4, 0) = 0;          //p3  
}

void outputCameraParam(void)
{
	/*保存数据*/
	//cvSave("cameraMatrix.xml", &intrinsic);  
	//cvSave("cameraDistoration.xml", &distortion_coeff);  
	//cvSave("rotatoVector.xml", &rvecs);  
	//cvSave("translationVector.xml", &tvecs);  
	/*输出数据*/
	cout << "fx :" << intrinsic.at<double>(0, 0) << endl << "fy :" << intrinsic.at<double>(1, 1) << endl;
	cout << "cx :" << intrinsic.at<double>(0, 2) << endl << "cy :" << intrinsic.at<double>(1, 2) << endl;

	cout << "k1 :" << distortion_coeff.at<double>(0, 0) << endl;
	cout << "k2 :" << distortion_coeff.at<double>(1, 0) << endl;
	cout << "p1 :" << distortion_coeff.at<double>(2, 0) << endl;
	cout << "p2 :" << distortion_coeff.at<double>(3, 0) << endl;
	cout << "p3 :" << distortion_coeff.at<double>(4, 0) << endl;
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR   lpCmdLine, _In_ int  nCmdShow)
{
	//string pic_name = "F:\\images\\06.jpg";

	//VideoCapture capture(1);//打开摄像头  

	//if (!capture.isOpened())//没有打开摄像头的话，就返回。
	//	return -1;
	//Mat edges; //定义一个Mat变量，循环显示每一帧t图像

	//Mat frame; //定义一个Mat变量，用于存储每一帧的图像  
	//capture >> frame;  //读取当前帧                          
	//if (frame.empty())
	//{
	//	return -1;
	//}
	//else
	//{
	//	cvtColor(frame, edges, CV_BGR2GRAY);//彩色转换成灰度  
	//	blur(edges, edges, Size(7, 7));//模糊化  
	//	Canny(edges, edges, 0, 30, 3);//边缘化  
	//	imshow("Video", frame); //显示当前帧  
	//	imwrite("F:\\images\\4.jpg", frame);//当前帧保存到文件

	//	waitKey(30); //延时30ms  
	//}




	int goodFrameCount = 0;

	while (goodFrameCount < frameNumber)
	{


		char filename[100];
		sprintf_s(filename, "F:\\images\\4.jpg", goodFrameCount + 1);
		
		rgbImage = imread(filename);
		namedWindow("window");
		imshow("window", rgbImage);
		cvtColor(rgbImage, grayImage, CV_BGR2GRAY);

		bool isFind = findChessboardCorners(rgbImage, boardSize, corner, 0);
		if (isFind == true) //所有角点都被找到 说明这幅图像是可行的  
		{
			/*
			Size(5,5) 搜索窗口的一半大小
			Size(-1,-1) 死区的一半尺寸
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件
			*/
			cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rgbImage, boardSize, corner, isFind);
			//imshow("chessboard", rgbImage);
			corners.push_back(corner);

			goodFrameCount++;
			cout << "The image is good" << endl;
		}
		else
		{
			cout << "The image is bad please try again" << endl;
		}


		if (waitKey(10) == 27)
		{
			break;
		}

	}

	/*
	图像采集完毕 接下来开始摄像头的校正
	calibrateCamera()
	输入参数 objectPoints  角点的实际物理坐标
	imagePoints   角点的图像坐标
	imageSize     图像的大小
	输出参数
	cameraMatrix  相机的内参矩阵
	distCoeffs    相机的畸变参数         
	rvecs         旋转矢量(外参数)
	tvecs         平移矢量(外参数）
	*/

	//intrinsic.create(3, 3, CV_64FC1);
	//distortion_coeff.create(5, 1, CV_64FC1);

	/*设置实际初始参数 根据calibrateCamera来 如果flag = 0 也可以不进行设置*/
	guessCameraParam();

	//MessageBox(NULL, _T("设置实际初始参数 successful"), _T("结果"), MB_OK | MB_ICONWARNING);
	/*计算实际的校正点的三维坐标*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	//MessageBox(NULL, _T("计算实际的校正点的三维坐标 successful"), _T("结果"), MB_OK | MB_ICONWARNING);
	/*标定摄像头*/
	//得到相机内部矩阵和畸变矩阵，以用来矫正
	calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
	//MessageBox(NULL, _T("标定摄像头successful"), _T("结果"), MB_OK | MB_ICONWARNING);
	/*保存并输出参数*/
	outputCameraParam();

	FileStorage fs("F:\\images\\intrinsic.xml", FileStorage::WRITE);
	fs << "intrinsic" << intrinsic;
	fs.release();

	FileStorage fs2("F:\\images\\distortion_coeff.xml", FileStorage::WRITE);
	fs2 << "distortion_coeff" << distortion_coeff;
	fs2.release();



	//waitKey(0);
	/*显示畸变校正效果*/
	//矫正

	Mat cImage;
	undistort(rgbImage, cImage, intrinsic, distortion_coeff);
	imshow("Corret Image", cImage);
	waitKey(0);
	//system("pause");
	return 0;
}


//#define calibration
//
//int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR   lpCmdLine, _In_ int  nCmdShow)
//{
//	ifstream fin("F:\\images\\1.jpg");             /* 标定所用图像文件的路径 */
//	ofstream fout("caliberation_result_right.txt");  /* 保存标定结果的文件 */
//
//													 // 读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
//	int image_count = 0;  /* 图像数量 */
//	Size image_size;      /* 图像的尺寸 */
//	Size board_size = Size(11, 8);             /* 标定板上每行、列的角点数 */
//	vector<Point2f> image_points_buf;         /* 缓存每幅图像上检测到的角点 */
//	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
//	string filename = "F:\\images\\1.jpg";      // 图片名
//	vector<string> filenames;
//
//	while (!filename.empty())
//	{
//		++image_count;
//		Mat imageInput = imread(filename);
//		filenames.push_back(filename);
//
//		// 读入第一张图片时获取图片大小
//		if (image_count == 1)
//		{
//			image_size.width = imageInput.cols;
//			image_size.height = imageInput.rows;
//		}
//
//		/* 提取角点 */
//		if (findChessboardCorners(imageInput, board_size, image_points_buf))
//		{
//			//cout << "can not find chessboard corners!\n";  // 找不到角点
//			cout << "**" << filename << "** can not find chessboard corners!\n";
//			exit(1);
//		}
//		else
//		{
//			Mat view_gray;
//			cvtColor(imageInput, view_gray, CV_RGB2GRAY);  // 转灰度图
//
//														   /* 亚像素精确化 */
//														   // image_points_buf 初始的角点坐标向量，同时作为亚像素坐标位置的输出
//														   // Size(5,5) 搜索窗口大小
//														   // （-1，-1）表示没有死区
//														   // TermCriteria 角点的迭代过程的终止条件, 可以为迭代次数和角点精度两者的组合
//			cornerSubPix(view_gray, image_points_buf, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//
//			image_points_seq.push_back(image_points_buf);  // 保存亚像素角点
//
//														   /* 在图像上显示角点位置 */
//			drawChessboardCorners(view_gray, board_size, image_points_buf, false); // 用于在图片中标记角点
//
//			imshow("Camera Calibration", view_gray);       // 显示图片
//
//			waitKey(500); //暂停0.5S      
//		}
//	}
//	int CornerNum = board_size.width * board_size.height;  // 每张图片上总的角点数
//
//														   //-------------以下是摄像机标定------------------
//
//														   /*棋盘三维信息*/
//	Size square_size = Size(10, 10);         /* 实际测量得到的标定板上每个棋盘格的大小 */
//	vector<vector<Point3f>> object_points;   /* 保存标定板上角点的三维坐标 */
//
//											 /*内外参数*/
//	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* 摄像机内参数矩阵 */
//	vector<int> point_counts;   // 每幅图像中角点的数量
//	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));       /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
//	vector<Mat> tvecsMat;      /* 每幅图像的旋转向量 */
//	vector<Mat> rvecsMat;      /* 每幅图像的平移向量 */
//
//							   /* 初始化标定板上角点的三维坐标 */
//	int i, j, t;
//	for (t = 0; t<image_count; t++)
//	{
//		vector<Point3f> tempPointSet;
//		for (i = 0; i<board_size.height; i++)
//		{
//			for (j = 0; j<board_size.width; j++)
//			{
//				Point3f realPoint;
//
//				/* 假设标定板放在世界坐标系中z=0的平面上 */
//				realPoint.x = i * square_size.width;
//				realPoint.y = j * square_size.height;
//				realPoint.z = 0;
//				tempPointSet.push_back(realPoint);
//			}
//		}
//		object_points.push_back(tempPointSet);
//	}
//
//	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
//	for (i = 0; i<image_count; i++)
//	{
//		point_counts.push_back(board_size.width * board_size.height);
//	}
//
//	/* 开始标定 */
//	// object_points 世界坐标系中的角点的三维坐标
//	// image_points_seq 每一个内角点对应的图像坐标点
//	// image_size 图像的像素尺寸大小
//	// cameraMatrix 输出，内参矩阵
//	// distCoeffs 输出，畸变系数
//	// rvecsMat 输出，旋转向量
//	// tvecsMat 输出，位移向量
//	// 0 标定时所采用的算法
//	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
//
//	//------------------------标定完成------------------------------------
//
//	// -------------------对标定结果进行评价------------------------------
//
//	double total_err = 0.0;         /* 所有图像的平均误差的总和 */
//	double err = 0.0;               /* 每幅图像的平均误差 */
//	vector<Point2f> image_points2;  /* 保存重新计算得到的投影点 */
//	fout << "每幅图像的标定误差：\n";
//
//	for (i = 0; i<image_count; i++)
//	{
//		vector<Point3f> tempPointSet = object_points[i];
//
//		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
//		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
//
//		/* 计算新的投影点和旧的投影点之间的误差*/
//		vector<Point2f> tempImagePoint = image_points_seq[i];
//		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
//		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
//
//		for (int j = 0; j < tempImagePoint.size(); j++)
//		{
//			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
//			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
//		}
//		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
//		total_err += err /= point_counts[i];
//		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
//	}
//	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
//
//	//-------------------------评价完成---------------------------------------------
//
//	//-----------------------保存定标结果------------------------------------------- 
//	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* 保存每幅图像的旋转矩阵 */
//	fout << "相机内参数矩阵：" << endl;
//	fout << cameraMatrix << endl << endl;
//	fout << "畸变系数：\n";
//	fout << distCoeffs << endl << endl << endl;
//	for (int i = 0; i<image_count; i++)
//	{
//		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
//		fout << tvecsMat[i] << endl;
//
//		/* 将旋转向量转换为相对应的旋转矩阵 */
//		Rodrigues(tvecsMat[i], rotation_matrix);
//		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
//		fout << rotation_matrix << endl;
//		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
//		fout << rvecsMat[i] << endl << endl;
//	}
//	fout << endl;
//
//	//--------------------标定结果保存结束-------------------------------
//
//	//----------------------显示定标结果--------------------------------
//
//	Mat mapx = Mat(image_size, CV_32FC1);
//	Mat mapy = Mat(image_size, CV_32FC1);
//	Mat R = Mat::eye(3, 3, CV_32F);
//	string imageFileName;
//	std::stringstream StrStm;
//	for (int i = 0; i != image_count; i++)
//	{
//		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
//		Mat imageSource = imread(filenames[i]);
//		Mat newimage = imageSource.clone();
//		remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
//		StrStm.clear();
//		imageFileName.clear();
//		StrStm << i + 1;
//		StrStm >> imageFileName;
//		imageFileName += "_d.jpg";
//		imwrite(imageFileName, newimage);
//	}
//
//	fin.close();
//	fout.close();
//
//
///// 读取一副图片，不改变图片本身的颜色类型（该读取方式为DOS运行模式）
//Mat src = imread("F:\\lane_line_detection\\left_img\\1.jpg");
//Mat distortion = src.clone();
//Mat camera_matrix = Mat(3, 3, CV_32FC1);
//Mat distortion_coefficients;
//
//
////导入相机内参和畸变系数矩阵
//FileStorage file_storage("F:\\lane_line_detection\\left_img\\Intrinsic.xml", FileStorage::READ);
//file_storage["CameraMatrix"] >> camera_matrix;
//file_storage["Dist"] >> distortion_coefficients;
//file_storage.release();
//
////矫正
//cv::undistort(src, distortion, camera_matrix, distortion_coefficients);
//
//cv::imshow("img", src);
//cv::imshow("undistort", distortion);
//cv::imwrite("undistort.jpg", distortion);
//
//cv::waitKey(0);
//
//return 0;
//}




