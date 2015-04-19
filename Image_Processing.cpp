#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

uchar ball_color[3][2] = {{'<', 60}, {'<', 60}, {'<', 60}}; //Stores the ball colour threshold in BGR and
															//condition as per the given ball in bgr value
															//can be changed according to the given ball

float threshold_standard_deviation = 20; 					//Set the threshold for checking whether the
															//given boundry is a circle or not

float threshold_average_distance_from_center = 25;			//Set the threshold for checking whether the
															//circle is big enough

int total_circles_detected = 0; 							//Will store the total number of detected circles finally

bool whether_any_object_detected = 0;

struct pixel
{
	int x;
	int y;

	void set(int r, int c)
	{
		x = r;
		y = c;
	}
};

float distance_pixels(pixel p1, pixel p2)
{
	float distance_x = pow((p1.x-p2.x), 2);
	float distance_y = pow((p1.y-p2.y), 2);
	float distance = sqrt(distance_x + distance_y);
	return distance;
}

struct boundry //This will store all the information of a blob detected
{
	bool whether_object; //set this to 0 if the blob can't be a cicle
	pixel boundry[5000];
	int total_boundry_pixels;
	pixel center;
	float distance_from_center[5000];
	float average_distance_from_center;
	float standard_deviation;

	void set_center()							//sets the center of the calculated boundry of the blob
	{
		center.set(0,0);
		for(int i=0; i<total_boundry_pixels; ++i)
		{
			center.x += boundry[i].x;
			center.y += boundry[i].y;
		}
		center.x /= total_boundry_pixels;
		center.y /= total_boundry_pixels;
	}

	void set_distance_from_center()				//Intializes the distance_from_center array & calculates avg distance
	{
		average_distance_from_center = 0;
		for(int i=0; i<total_boundry_pixels; ++i)
		{
			distance_from_center[i] = distance_pixels(center, boundry[i]);
			average_distance_from_center += distance_from_center[i];
		}
		average_distance_from_center /= total_boundry_pixels;
	}


	float calculate_standard_deviation()		//Calcualtes standard deviation of the distances
	{
		standard_deviation = 0;
		for(int i = 0; i < total_boundry_pixels; ++i)
		{
			standard_deviation += pow((average_distance_from_center - distance_from_center[i]),2);
		}
		standard_deviation /= total_boundry_pixels;
		standard_deviation = sqrt(standard_deviation);
	}
	void check_whether_circle()					//Checks whether the boundry is an approximate circle
	{
		set_center();
		set_distance_from_center();

		if(average_distance_from_center < threshold_average_distance_from_center)
		{
			whether_object = false;
			return;
		}
		else
			whether_object = true;
		calculate_standard_deviation();
		if(standard_deviation < threshold_standard_deviation)
			whether_object = true;
		else
			whether_object = false;

		if (whether_object == true)
			total_circles_detected++;
	}

};

boundry boundry[10000];

int total_boundries = 0;

pixel surrounding[8] = {{0,1},{0,-1},{1,1},{1,-1},{1,0},{-1,-1},{-1,0},{-1,1}};

bool satisfy_ballcolor(Vec3b bgr_value)
{
	bool return_value = 1;
	for(int i = 0; i<3; ++i) //For B, G, R
	{
		switch(ball_color[i][0])
		{
		    case '~':   if(ball_color[i][1]-10 > bgr_value[i] || ball_color[i][1] + 10 < bgr_value[i])
                        return_value = 0;

			case '<':	if(bgr_value[i] >= ball_color[i][1])
						return_value = 0;
						break;

			case '>':	if(bgr_value[i] <= ball_color[i][1])
						return_value = 0;
						break;

			case '=':	if(bgr_value[i] != ball_color[i][1])
						return_value = 0;
						break;

			default:	break;
		}
	}
	return return_value;
}

void convert_only_ballcolor(const Mat &original_image, Mat &only_ballcolor)
{
	Vec3b bgr_value;
	for(int i=0; i<original_image.rows; ++i)
		for(int j=0; j<original_image.cols; ++j)
		{
			bgr_value = original_image.at<Vec3b>(i, j);
			if(satisfy_ballcolor(bgr_value))
			{
				only_ballcolor.at<uchar>(i, j) = 0 ;
			}
		}
}

void check_for_circles(struct boundry boundry[], int total_boundries)
{
	total_circles_detected = 0;
	for(int i=0; i<total_boundries; ++i)
	{
		boundry[i].check_whether_circle();
	}
}

void draw_detected_object(Mat &img, struct boundry boundry[], int total_boundries)
{
	for(int i = 0; i < total_boundries; ++i)
	{
		if(boundry[i].whether_object)
		{
			for(int j = 0; j < boundry[i].total_boundry_pixels; ++j)
			{
				img.at<uchar>(boundry[i].boundry[j].x, boundry[i].boundry[j].y) = 0;
			}
		img.at<uchar>(boundry[i].center.x, boundry[i].center.y)=0;
	}
	}
}

void detect_all_boundries(Mat &img, const Mat &only_ballcolor)
{
	int total_black_surrounding_pixels;
	for(int i = 0; i < img.rows; ++i)
		for(int j = 0; j < img.cols; ++j)
		{
			if(only_ballcolor.at<uchar>(i, j)==0)
			{
				total_black_surrounding_pixels = 0;
				for(int k = 0; k < 8; ++k)
				{
					if(only_ballcolor.at<uchar>(i + surrounding[k].x, j + surrounding[k].y)==0)
						total_black_surrounding_pixels++;
				}
				if(total_black_surrounding_pixels < 8)
				{
					img.at<uchar>(i, j) = 0;
				}
			}
		}
}

void store_a_boundary(struct boundry &boundry, Mat &img, int r, int c)
{
    pixel current_frontier[10000], next_frontier[10000];
	int total_pixels_in_current_frontier = 1, total_pixels_in_next_frontier = 0;
	img.at<uchar>(r, c) = 255;
	current_frontier[0].set(r, c);
	boundry.total_boundry_pixels = 0;
	while(total_pixels_in_current_frontier != 0)
	{
		for(int i = 0; i < total_pixels_in_current_frontier; ++i)
		{
			for(int j = 0; j < 8; ++j)
			{
				if(current_frontier[i].x + surrounding[j].x < img.rows
					&& current_frontier[i].y + surrounding[j].y < img.cols
					&& current_frontier[i].x + surrounding[j].x >=0
					&& current_frontier[i].y + surrounding[j].y >= 0)
				{
					if(img.at<uchar>(current_frontier[i].x + surrounding[j].x, current_frontier[i].y + surrounding[j].y) == 0)
					{
						img.at<uchar>(current_frontier[i].x + surrounding[j].x, current_frontier[i].y + surrounding[j].y) = 255;
						next_frontier[total_pixels_in_next_frontier].set(current_frontier[i].x + surrounding[j].x, current_frontier[i].y + surrounding[j].y);
						total_pixels_in_next_frontier++;
					}
				}
			}
		}
		for(int i = 0; i < total_pixels_in_current_frontier; ++i)
		{
			boundry.boundry[boundry.total_boundry_pixels] = current_frontier[i];
			boundry.total_boundry_pixels++;
		}
		total_pixels_in_current_frontier = total_pixels_in_next_frontier;
		for(int i = 0; i < total_pixels_in_current_frontier; ++i)
		{
			current_frontier[i] = next_frontier[i];
		}
		total_pixels_in_next_frontier = 0;
	}
}

void store_all_boundries(struct boundry boundry[], int &total_boundries, Mat &img)
{
	total_boundries = 0;
	for(int i = 0; i < img.rows; i++)
		for(int j = 0; j < img.cols; j++)
		{
			if(img.at<uchar>(i,j)==0)
			{
				store_a_boundary(boundry[total_boundries], img, i, j);
				total_boundries++;
			}
		}
}

void check_number_of_circles_detected()
{
	if(total_circles_detected!=1)						//If no or more than 1 circle detected
	{
		total_boundries = 0; 							//So that no further action is taken
														//For drawing or sending data we run a loop from 0 to total_boundries
	}
}



int main()
{
    VideoCapture camera(0);
    Mat original_image;
    if(!camera.isOpened())
    {
        cout<<"Can't open Camera!"<<endl;
        return -1;
    }
    namedWindow("original_image", CV_WINDOW_NORMAL);
    namedWindow("only_ballcolor", CV_WINDOW_NORMAL);
    namedWindow("only_boundries", CV_WINDOW_NORMAL);
    namedWindow("detected_object", CV_WINDOW_NORMAL);
    while(1)
    {
        if(!camera.read(original_image))
        {
            cout<<"Can't read frames from camera.";
            return -1;
        }
        camera.read(original_image);
        imshow("original_image", original_image);
        GaussianBlur(original_image, original_image, Size(13, 13), 0, 0 );
        Mat only_ballcolor(original_image.rows, original_image.cols, CV_8UC1, 255);
        convert_only_ballcolor(original_image, only_ballcolor);
        imshow("only_ballcolor", only_ballcolor);
        Mat only_boundries(original_image.rows, original_image.cols, CV_8UC1, 255);
        detect_all_boundries(only_boundries, only_ballcolor);
        imshow("only_boundries", only_boundries);
        Mat detected_object = only_boundries.clone();
        store_all_boundries(boundry, total_boundries, detected_object);
        check_for_circles(boundry, total_boundries);
        check_number_of_circles_detected();
        draw_detected_object(detected_object, boundry, total_boundries);
        imshow("detected_object", detected_object);
        if(waitKey(1)==27)
        {
            destroyWindow("original_image");
            destroyWindow("only_ballcolor");
            destroyWindow("only_boundries");
            destroyWindow("detected_object");
            return 1;
        }
    }
}

