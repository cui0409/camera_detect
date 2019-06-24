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


const int imageWidth = 640;                             //����ͷ�ķֱ���  
const int imageHeight = 480;
const int boardWidth = 11;                               //����Ľǵ���Ŀ  
const int boardHeight = 8;                              //����Ľǵ�����  
const int boardCorner = boardWidth * boardHeight;       //�ܵĽǵ�����  
const int frameNumber = 5;                             //����궨ʱ��Ҫ���õ�ͼ��֡��  
const int squareSize = 10;                              //�궨��ڰ׸��ӵĴ�С ��λmm  
const Size boardSize = Size(boardWidth, boardHeight);   //  

Mat intrinsic;                                          //����ڲ���  
Mat distortion_coeff;                                   //����������  
vector<Mat> rvecs;                                        //��ת����  
vector<Mat> tvecs;                                        //ƽ������  
vector<vector<Point2f>> corners;                        //����ͼ���ҵ��Ľǵ�ļ��� ��objRealPoint һһ��Ӧ  
vector<vector<Point3f>> objRealPoint;                   //����ͼ��Ľǵ��ʵ���������꼯��  


vector<Point2f> corner;                                   //ĳһ��ͼ���ҵ��Ľǵ�  

Mat rgbImage, grayImage;

/*����궨����ģ���ʵ����������*/
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

/*��������ĳ�ʼ���� Ҳ���Բ�����*/
void guessCameraParam(void)
{
	/*�����ڴ�*/
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
	/*��������*/
	//cvSave("cameraMatrix.xml", &intrinsic);  
	//cvSave("cameraDistoration.xml", &distortion_coeff);  
	//cvSave("rotatoVector.xml", &rvecs);  
	//cvSave("translationVector.xml", &tvecs);  
	/*�������*/
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

	//VideoCapture capture(1);//������ͷ  

	//if (!capture.isOpened())//û�д�����ͷ�Ļ����ͷ��ء�
	//	return -1;
	//Mat edges; //����һ��Mat������ѭ����ʾÿһ֡tͼ��

	//Mat frame; //����һ��Mat���������ڴ洢ÿһ֡��ͼ��  
	//capture >> frame;  //��ȡ��ǰ֡                          
	//if (frame.empty())
	//{
	//	return -1;
	//}
	//else
	//{
	//	cvtColor(frame, edges, CV_BGR2GRAY);//��ɫת���ɻҶ�  
	//	blur(edges, edges, Size(7, 7));//ģ����  
	//	Canny(edges, edges, 0, 30, 3);//��Ե��  
	//	imshow("Video", frame); //��ʾ��ǰ֡  
	//	imwrite("F:\\images\\4.jpg", frame);//��ǰ֡���浽�ļ�

	//	waitKey(30); //��ʱ30ms  
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
		if (isFind == true) //���нǵ㶼���ҵ� ˵�����ͼ���ǿ��е�  
		{
			/*
			Size(5,5) �������ڵ�һ���С
			Size(-1,-1) ������һ��ߴ�
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)������ֹ����
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
	ͼ��ɼ���� ��������ʼ����ͷ��У��
	calibrateCamera()
	������� objectPoints  �ǵ��ʵ����������
	imagePoints   �ǵ��ͼ������
	imageSize     ͼ��Ĵ�С
	�������
	cameraMatrix  ������ڲξ���
	distCoeffs    ����Ļ������         
	rvecs         ��תʸ��(�����)
	tvecs         ƽ��ʸ��(�������
	*/

	//intrinsic.create(3, 3, CV_64FC1);
	//distortion_coeff.create(5, 1, CV_64FC1);

	/*����ʵ�ʳ�ʼ���� ����calibrateCamera�� ���flag = 0 Ҳ���Բ���������*/
	guessCameraParam();

	//MessageBox(NULL, _T("����ʵ�ʳ�ʼ���� successful"), _T("���"), MB_OK | MB_ICONWARNING);
	/*����ʵ�ʵ�У�������ά����*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	//MessageBox(NULL, _T("����ʵ�ʵ�У�������ά���� successful"), _T("���"), MB_OK | MB_ICONWARNING);
	/*�궨����ͷ*/
	//�õ�����ڲ�����ͻ����������������
	calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
	//MessageBox(NULL, _T("�궨����ͷsuccessful"), _T("���"), MB_OK | MB_ICONWARNING);
	/*���沢�������*/
	outputCameraParam();

	FileStorage fs("F:\\images\\intrinsic.xml", FileStorage::WRITE);
	fs << "intrinsic" << intrinsic;
	fs.release();

	FileStorage fs2("F:\\images\\distortion_coeff.xml", FileStorage::WRITE);
	fs2 << "distortion_coeff" << distortion_coeff;
	fs2.release();



	//waitKey(0);
	/*��ʾ����У��Ч��*/
	//����

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
//	ifstream fin("F:\\images\\1.jpg");             /* �궨����ͼ���ļ���·�� */
//	ofstream fout("caliberation_result_right.txt");  /* ����궨������ļ� */
//
//													 // ��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��
//	int image_count = 0;  /* ͼ������ */
//	Size image_size;      /* ͼ��ĳߴ� */
//	Size board_size = Size(11, 8);             /* �궨����ÿ�С��еĽǵ��� */
//	vector<Point2f> image_points_buf;         /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
//	vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */
//	string filename = "F:\\images\\1.jpg";      // ͼƬ��
//	vector<string> filenames;
//
//	while (!filename.empty())
//	{
//		++image_count;
//		Mat imageInput = imread(filename);
//		filenames.push_back(filename);
//
//		// �����һ��ͼƬʱ��ȡͼƬ��С
//		if (image_count == 1)
//		{
//			image_size.width = imageInput.cols;
//			image_size.height = imageInput.rows;
//		}
//
//		/* ��ȡ�ǵ� */
//		if (findChessboardCorners(imageInput, board_size, image_points_buf))
//		{
//			//cout << "can not find chessboard corners!\n";  // �Ҳ����ǵ�
//			cout << "**" << filename << "** can not find chessboard corners!\n";
//			exit(1);
//		}
//		else
//		{
//			Mat view_gray;
//			cvtColor(imageInput, view_gray, CV_RGB2GRAY);  // ת�Ҷ�ͼ
//
//														   /* �����ؾ�ȷ�� */
//														   // image_points_buf ��ʼ�Ľǵ�����������ͬʱ��Ϊ����������λ�õ����
//														   // Size(5,5) �������ڴ�С
//														   // ��-1��-1����ʾû������
//														   // TermCriteria �ǵ�ĵ������̵���ֹ����, ����Ϊ���������ͽǵ㾫�����ߵ����
//			cornerSubPix(view_gray, image_points_buf, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//
//			image_points_seq.push_back(image_points_buf);  // ���������ؽǵ�
//
//														   /* ��ͼ������ʾ�ǵ�λ�� */
//			drawChessboardCorners(view_gray, board_size, image_points_buf, false); // ������ͼƬ�б�ǽǵ�
//
//			imshow("Camera Calibration", view_gray);       // ��ʾͼƬ
//
//			waitKey(500); //��ͣ0.5S      
//		}
//	}
//	int CornerNum = board_size.width * board_size.height;  // ÿ��ͼƬ���ܵĽǵ���
//
//														   //-------------������������궨------------------
//
//														   /*������ά��Ϣ*/
//	Size square_size = Size(10, 10);         /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
//	vector<vector<Point3f>> object_points;   /* ����궨���Ͻǵ����ά���� */
//
//											 /*�������*/
//	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* ������ڲ������� */
//	vector<int> point_counts;   // ÿ��ͼ���нǵ������
//	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));       /* �������5������ϵ����k1,k2,p1,p2,k3 */
//	vector<Mat> tvecsMat;      /* ÿ��ͼ�����ת���� */
//	vector<Mat> rvecsMat;      /* ÿ��ͼ���ƽ������ */
//
//							   /* ��ʼ���궨���Ͻǵ����ά���� */
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
//				/* ����궨�������������ϵ��z=0��ƽ���� */
//				realPoint.x = i * square_size.width;
//				realPoint.y = j * square_size.height;
//				realPoint.z = 0;
//				tempPointSet.push_back(realPoint);
//			}
//		}
//		object_points.push_back(tempPointSet);
//	}
//
//	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
//	for (i = 0; i<image_count; i++)
//	{
//		point_counts.push_back(board_size.width * board_size.height);
//	}
//
//	/* ��ʼ�궨 */
//	// object_points ��������ϵ�еĽǵ����ά����
//	// image_points_seq ÿһ���ڽǵ��Ӧ��ͼ�������
//	// image_size ͼ������سߴ��С
//	// cameraMatrix ������ڲξ���
//	// distCoeffs ���������ϵ��
//	// rvecsMat �������ת����
//	// tvecsMat �����λ������
//	// 0 �궨ʱ�����õ��㷨
//	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
//
//	//------------------------�궨���------------------------------------
//
//	// -------------------�Ա궨�����������------------------------------
//
//	double total_err = 0.0;         /* ����ͼ���ƽ�������ܺ� */
//	double err = 0.0;               /* ÿ��ͼ���ƽ����� */
//	vector<Point2f> image_points2;  /* �������¼���õ���ͶӰ�� */
//	fout << "ÿ��ͼ��ı궨��\n";
//
//	for (i = 0; i<image_count; i++)
//	{
//		vector<Point3f> tempPointSet = object_points[i];
//
//		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
//		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
//
//		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
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
//		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
//	}
//	fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
//
//	//-------------------------�������---------------------------------------------
//
//	//-----------------------���涨����------------------------------------------- 
//	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* ����ÿ��ͼ�����ת���� */
//	fout << "����ڲ�������" << endl;
//	fout << cameraMatrix << endl << endl;
//	fout << "����ϵ����\n";
//	fout << distCoeffs << endl << endl << endl;
//	for (int i = 0; i<image_count; i++)
//	{
//		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
//		fout << tvecsMat[i] << endl;
//
//		/* ����ת����ת��Ϊ���Ӧ����ת���� */
//		Rodrigues(tvecsMat[i], rotation_matrix);
//		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
//		fout << rotation_matrix << endl;
//		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
//		fout << rvecsMat[i] << endl << endl;
//	}
//	fout << endl;
//
//	//--------------------�궨����������-------------------------------
//
//	//----------------------��ʾ������--------------------------------
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
///// ��ȡһ��ͼƬ�����ı�ͼƬ�������ɫ���ͣ��ö�ȡ��ʽΪDOS����ģʽ��
//Mat src = imread("F:\\lane_line_detection\\left_img\\1.jpg");
//Mat distortion = src.clone();
//Mat camera_matrix = Mat(3, 3, CV_32FC1);
//Mat distortion_coefficients;
//
//
////��������ڲκͻ���ϵ������
//FileStorage file_storage("F:\\lane_line_detection\\left_img\\Intrinsic.xml", FileStorage::READ);
//file_storage["CameraMatrix"] >> camera_matrix;
//file_storage["Dist"] >> distortion_coefficients;
//file_storage.release();
//
////����
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




