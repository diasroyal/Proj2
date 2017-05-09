// USAGE: ./tracking <video_path> <fps> <frame_window_size>

/* **************************************************************************************** 
* This code was adapted from KLT tracker example from OpenCV Library:
* https://github.com/Itseez/opencv/blob/master/samples/cpp/lkdemo.cpp
*
* We took the lkdemo.cpp as skeleton code and built on top of that our specific application
**************************************************************************************** */


/* ***** START POINT OF CODE THAT BELONGS TO lkdemo.cpp ****** */

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

int save_output_video = 1;

static void help()
{
    // print a welcome message, and the OpenCV version
	cout << "\nThis is a program that detects the frequency of objects that vibrate at a constant rate.";
	cout << " The Lukas-Kanade optical flow lkdemo() is used, and the OpenCV library version is: " << CV_VERSION << endl;
	cout << "\t USAGE: ./tracking <video_path> <sampling_frequency_of_input_video> (optional)<frame_window_size> \n";
	cout << "\t Usage example: ./tracking tuning_fork.avi 1200 200 \n";
	cout << "\nHot keys: \n"
	"\tESC - quit the program\n"
	"\tr - auto-initialize tracking\n"
	"\tc - delete all the points\n"
	"To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
	if( event == EVENT_LBUTTONDOWN )
	{
		point = Point2f((float)x, (float)y);
		addRemovePt = true;
	}
}



int main( int argc, char** argv )
{

	if (argc < 3)
	{
		cout << "Not enough parameters" << endl;
		help();
		return 0;
	}

	string input 	= argv[1];
	string fps_s 	= argv[2];
	double fps 		= atoi(fps_s.c_str()); 

	bool needToInit     = true;
	const int MAX_COUNT = 500;
	int frame_window 	= 100;

	if (argc > 3)
	{
		string frame_window_s = argv[3];
		frame_window = atoi(frame_window_s.c_str());
	}


	VideoCapture cap;
	TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
	Size subPixWinSize(10,10), winSize(31,31);


	if( input.empty() )
		cap.open(0);
	else if( input.size() == 1 && isdigit(input[0]) )
		cap.open(input[0] - '0');
	else
		cap.open(input);

	if( !cap.isOpened() )
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	namedWindow( "Vibrations", 1 );
	setMouseCallback( "Vibrations", onMouse, 0 );
/* ***** END POINT OF CODE THAT BELONGS TO lkdemo.cpp ****** */

	Mat gray, prevGray, image, frame;
	vector<Point2f> points[2];
	vector<bool> prevless;
	vector<double> countofpeaks;
	vector<float> max_y;
	vector<float> min_y;
	vector<float> distance;
	vector<bool>red;
	float max_distance = 0;
	int no_of_frames_elapsed = 0;


	map<double, double> freq_histogram;
	vector<double> temporal_frequency;
	string freq_s;


	int height = (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	int width = (int) cap.get(CV_CAP_PROP_FRAME_WIDTH);

    /* set width and height if video is bigger that 480x240 or 240x480 */
	if(height > width)
	{ 
		height = (height > 480) ? 480 : height;
		width = (width > 240) ? 240 : width;
	}
	else 
	{
		height = (height > 240) ? 240 : height;
		width = (width > 480) ? 480 : width;
	}
	cv::Size S = cv::Size(width, height);

	/* Create a video output for the results */
	std::string output_video_file = input + "_output.avi";
	cv::VideoWriter output_video;
	int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
	output_video.open(output_video_file, CV_FOURCC('8','B','P','S'), (int) fps, S, true);

	for(;;)
	{
		no_of_frames_elapsed++;
		cap >> frame;

		/* // Discomment this if you want to lower the fps 
		cap >> frame;
		cap >> frame;
		cap >> frame;
		cap >> frame;
		*/
		
		if( frame.empty() ){
			cout << "empty frame" << endl;
			break;
		}

		// resize frame
		resize(frame, frame, S);
		frame.copyTo(image);
		cvtColor(image, gray, COLOR_BGR2GRAY);

/* ***** START POINT OF CODE THAT BELONGS TO lkdemo.cpp ****** */
		if( needToInit )
		{
            /* Determines strong corners on an image */
			goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);

            /* Refines the corner locations. The function iterates to find the sub-pixel accurate 
             * location of corners or radial saddle points */
			cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
			for(int i=0;i<points[1].size();i++)
			{
				prevless.push_back(false);
				red.push_back(false);
				countofpeaks.push_back(0);

			}
			addRemovePt = false;

		}
		else if( !points[0].empty() )
		{
			vector<uchar> status;
			vector<float> err;
			if(prevGray.empty())
				gray.copyTo(prevGray);

            /* Calculates an optical flow for a sparse feature set using the iterative Lucas-Kanade method with pyramids. */
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
			size_t i, k;
			for( i = k = 0; i < points[1].size(); i++ )
			{
				if( addRemovePt )
				{
					if( norm(point - points[1][i]) <= 5 )
					{
						addRemovePt = false;
						continue;
					}
				}

				if( !status[i] )
					continue;

				points[1][k++] = points[1][i];

				/* Draw circle points in output image */
				if(red[i])
					circle( image, points[1][i], 3, Scalar(0,0,255), -1, 8);
				else
					circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
			}
			points[1].resize(k);
		}

		if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
		{
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;   
		}
/* ***** END POINT OF CODE THAT BELONGS TO lkdemo.cpp ****** */

		if( !points[0].empty() ){

			std::vector<double> peaks;
			for(int i = 0; i < points[0].size(); i++)
			{
				/* Count peaks in the Y direction by checking the points p where p[i-1] < p[i] > p[i+1] */
				if(prevless[i] && points[0][i].y > points[1][i].y)
				{
					countofpeaks[i]++;
					prevless[i] = false;
				}
				if(points[0][i].y < points[1][i].y)
					prevless[i] = true;
				else
					prevless[i] = false;

				/* Initialize the distance vector */
				if(no_of_frames_elapsed == 2)
				{
					max_y.push_back(points[0][i].y);
					min_y.push_back(points[0][i].y);
					distance.push_back(0);
				}
				else
				{
					/* Calculate distance between points */
					if(points[1][i].y > max_y[i])
					{
						max_y[i] 	= points[1][i].y;
						distance[i] = max_y[i] - min_y[i];

						if(max_distance < distance[i])
							max_distance = distance[i];
					}
					if(points[1][i].y < min_y[i])
					{
						min_y[i] 	= points[1][i].y;
						distance[i] = max_y[i] - min_y[i];

						if(max_distance < distance[i])
							max_distance = distance[i];
					}
				}

				if(no_of_frames_elapsed == frame_window)
				{
					prevless[i] 			= false;
					double peaks_per_frame 	= (double)countofpeaks[i] / (double)no_of_frames_elapsed * fps;

					if(distance[i] >= (0.6 * max_distance))
					{
						red[i] = true;
						peaks.push_back(peaks_per_frame);
						cout << peaks_per_frame << ",";

						if ( freq_histogram.find(peaks_per_frame) == freq_histogram.end() )
							freq_histogram[ peaks_per_frame ] = 1;
						else 
							freq_histogram[ peaks_per_frame ] += 1;
					}
					countofpeaks[i] = 0;
				}
			}

			if (peaks.size() > 0)
			{
				double max = 0;
				double freq = 0;
				for(std::map<double, double>::iterator it = freq_histogram.begin(); it != freq_histogram.end(); it++) {

					if (it->second > max)
					{
						max 	= it->second;
						freq 	= it->first;
					}
				}

				cout << " freq " << freq << endl;
				temporal_frequency.push_back(freq);
				freq_s = to_string(freq);
			}
		}

		if(no_of_frames_elapsed == frame_window)
		{
			cout << endl;
			no_of_frames_elapsed 	= 0;
			max_distance 			= 0;
			max_y.clear();
			min_y.clear();
			distance.clear();
		}

		/* Write temporal frequency in output video */
		putText(image, freq_s.c_str(), cvPoint(30, 30), CV_FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1, CV_AA);
		
		if (output_video.isOpened() && save_output_video)
			output_video << image;
		
/* ***** START POINT OF CODE THAT BELONGS TO lkdemo.cpp ****** */
		imshow("Vibrations", image);
		
		needToInit = false;

		char c = (char)waitKey(10);
		if( c == 27 )
			break;
		switch( c )
		{
			case 'r':
			needToInit = true;
			break;
			case 'c':
			points[0].clear();
			points[1].clear();
			break;
		}

		std::swap(points[1], points[0]);

		cv::swap(prevGray, gray);
/* ***** END POINT OF CODE THAT BELONGS TO lkdemo.cpp ****** */

	}

	cout << "frame_window: " << frame_window << endl;
	cout << "temporal_frequency: ";
	for (int i = 0; i < temporal_frequency.size(); ++i)
	{
		cout << temporal_frequency[i] << " ";
	}
	cout << endl;

	cout << "frequency_histogram: " << endl;
	for(std::map< double, double >::iterator it = freq_histogram.begin(); it != freq_histogram.end(); it++) {
		cout << it->first << " " << it->second << endl;
	}

	return 0;
}