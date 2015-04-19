/********************************************//**
 * Written by: Group 6, CS101 Embedded Systems Project
 *
 * Date : 18-04-15
 *
 * This code demonstrates the processing of the image sent from the bot
 * (using the webcam) to see whether the required object is present in the
 * corresponding portion of the arena (whose image has been captured) and
 * send the required command for action to the bot.
 *
 * Note:
 * 1. "windows.h" file used for Xbee communication so the code can only be
 *    run on a windows OS
 * 2. For linking and installing OpenCV refer to the screencast video.
 ***********************************************/


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <windows.h>

/**Data Encryption
*
*    BOT TO PC:
*
*    0x00 - Process Image
*
*    0x80 - received data
*
*    PC TO BOT:
*
*    0x00 - Ball not found
*
*    0x1(sign)XXXXXX - Distance from center (Ball found)
*
*    0x03 - a byte received from XBEE
*/

using namespace std;
using namespace cv;

HANDLE hSerial = CreateFile("COM1", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
DCB dcb;

/**Stores the ball colour threshold in BGR and condition as per the given ball in bgr value
*can be changed according to the given ball
*/
uchar ball_color[3][2] = {{'<', 60}, {'<', 60}, {'<', 60}};

///Set the threshold for checking whether the given boundry is a circle or not
float threshold_standard_deviation = 20;

///Set the threshold for checking whether the circle is big enough
float threshold_average_distance_from_center = 25;

///Will store the total number of detected circles finally
int total_circles_detected = 0;

///Stores the bytes received from the Xbee on the bot
uchar input_data_from_xbee[2] = {0xFF,0};

///Stores the bytes to be written onto the port
uchar output_data_to_xbee[2] = {0x8E,0};

/// Stores the displacement of the bot in the X direction
int x_coordinate_of_bot = 0;

///Stores the displacement of the bot in the Y direction
int y_coordinate_of_bot = 0;

///True when sending X-coordinate is over and sending the Y-coordinate is being sent
bool whether_sending_x_coordinate_over = 0;

///Ture when the data being sent is the Y-coordinate of the bot
int is_the_coordinate_Y = 0;

///True when the object is detected
bool whether_any_object_detected = 0;

bool write_data_to_XBEE();

/// Stores the coordinate of a pixel
struct pixel
{
	int x;
	int y;

	///Function to initialize the variables of the struct "pixel"
	void set(int r, int c)
	{
		x = r;
		y = c;
	}
};

/**Precondition : p1 and p2 are variables of "pixel" type
*
*This function returns the distance between the two input pixels p1 and p2
*/
float distance_pixels(pixel p1, pixel p2)
{
	float distance_x = pow((p1.x-p2.x), 2);
	float distance_y = pow((p1.y-p2.y), 2);
	float distance = sqrt(distance_x + distance_y);
	return distance;
}

///This structure stores all the information of a detected blob
struct boundry
{
	///Set to 0 if the blob cannot be a circle
	bool whether_object;
	///Array which stores the pixels of the boundary of the setected blob
    pixel boundry[5000];
    ///Stores the number of boundary pixels of the detected blob
	int total_boundry_pixels;
	///Stores the center pixel of the detected blob
	pixel center;
	///Array which stores the distance of each point on the boundary from the centre of the blob
	float distance_from_center[5000];
	///Stores the average distance of the boundary points from the centre of the blob
	float average_distance_from_center;
	///Stores the standard deviation of the distance of each point on the boundary from the centre of the blob
	float standard_deviation;

	///Sets the center of the calculated boundry of the blob
	void set_center()
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

	///Intializes the distance_from_center array and calculates the average distance
	void set_distance_from_center()
	{
		average_distance_from_center = 0;
		for(int i=0; i<total_boundry_pixels; ++i)
		{
			distance_from_center[i] = distance_pixels(center, boundry[i]);
			average_distance_from_center += distance_from_center[i];
		}
		average_distance_from_center /= total_boundry_pixels;
	}


	///Calcualtes standard deviation of the distances of points on the boundary from the centre
	float calculate_standard_deviation()
	{
		standard_deviation = 0;
		for(int i = 0; i < total_boundry_pixels; ++i)
		{
			standard_deviation += pow((average_distance_from_center - distance_from_center[i]),2);
		}
		standard_deviation /= total_boundry_pixels;
		standard_deviation = sqrt(standard_deviation);
	}

	///Checks whether the boundary is an approximate circle
	void check_whether_circle()
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

///Array of type struct "boundry" which stores all closed boundaries detected in the image
boundry boundry[10000];

///Stores the number of boundaries detected in the image
int total_boundries = 0;

///Array of type "pixel" which defines the relative locations of the eight adjecent pixels to every pixel in the image
pixel surrounding[8] = {{0,1},{0,-1},{1,1},{1,-1},{1,0},{-1,-1},{-1,0},{-1,1}};

/** Precondition : "bgr_value" specifies the BGR value of the colour of the ball to be detected
*
*This function returns true if an object of the specified colur is found else returns false.
*/
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

/**Precondition : Takes as arguments a martix of the original image and the matrix storing pixels having the colour specified
*
*This function sets the pixels having the colour specified by the user to black and the other pixels to white
*in the matrix "only_ballcolor"
*/
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


/**Precondition : Takes as argument the structure storing the boundaries of the detected blobs and the total number of boundaries
*
*This function checks whether the detected boundaries are cicles or not
*/
void check_for_circles(struct boundry boundry[], int total_boundries)
{
	total_circles_detected = 0;
	for(int i=0; i<total_boundries; ++i)
	{
		boundry[i].check_whether_circle();
	}
}

///This function draws the detected object onto the matrix "img"
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

///This function detects the boundary point of the blobs
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

///This function finds boundaries and stores them into the struct "boundry"
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

///This function stores all the boundaries
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

/**This function checks if the number of circles detected is exactly one.
* If the number of detected circles is greater than one, it resets the totoal boundaries to zero
*/
void check_number_of_circles_detected()
{
	if(total_circles_detected!=1)						//If no or more than 1 circle detected
	{
		total_boundries = 0; 							//So that no further action is taken
														//For drawing or sending data we run a loop from 0 to total_boundries
	}
}

/// This function reads data from the Xbee and returns true if the data is received else returns false
bool read_data_from_XBEE()
{
    DWORD dwBytesTransferred = 0;
    DWORD dwCommModemStatus = 0;
    bool retVal;
    if(!GetCommState(hSerial, &dcb))
    {
        cout<<"Serial port can't be opened"<<endl;
        return 0;
    }

    SetCommMask(hSerial, EV_RXCHAR|EV_ERR);
    WaitCommEvent(hSerial, &dwCommModemStatus, 0);
    if (dwCommModemStatus& EV_RXCHAR)
    {
        retVal = (ReadFile(hSerial, input_data_from_xbee, 1, &dwBytesTransferred, NULL));
    }

    else
    {
        cout<<"Some error has occured"<<endl;
        return 0;
    }

    if(retVal && input_data_from_xbee[0] != 0x80)
    {
        output_data_to_xbee[0] = 0x03;
        write_data_to_XBEE();
        Sleep(10);
    }
    return retVal;
}

/// This function writes data from the Xbee and returns true if the data is sucessfully written to the port
bool write_data_to_XBEE()
{
    DWORD byteswritten;
    DWORD received_byte;
    uchar whether_received[2] = {0};
    unsigned long long timer1,timer2;
    bool has_the_received_byte_been_received = 0;

    if (!GetCommState(hSerial, &dcb))
    {
        cout<<"\n Serial port can't be opened"<<endl;
        return false;
    }
    Sleep(10);
    bool retVal;
    while(!has_the_received_byte_been_received)
    {
        timer1 = GetTickCount();
        timer2 = timer1;
        retVal = WriteFile(hSerial, output_data_to_xbee, 1, &byteswritten, NULL);
        while(timer2 - timer1 < 1000)
        {
            if(!ReadFile(hSerial, whether_received, 1, &received_byte, NULL));
            if(whether_received[0] == 0x80 || output_data_to_xbee[0] == 0x03)
            {
                has_the_received_byte_been_received = 1;
                whether_received[0] = 0;
                break;
            }
            timer2 = GetTickCount();
        }
    }
    return retVal;
}

///This function sets up serial communication between the bot and the laptop and returns true if successful
bool set_up_serial_connection()
{
    if (!GetCommState(hSerial, &dcb))
    {
        cout<<"Serial port can't be opened"<<endl;
        return false;
    }
    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    return SetCommState(hSerial, &dcb);
}

///This function sends the distance of the centre of the detected object from the centre of the image to the bot
void send_distance_from_center(int center_column_of_the_image)
{
    whether_any_object_detected = 0;
    int distance_of_the_center_of_the_object_from_the_center_of_the_image;
    for(int i = 0; i < total_boundries; ++i)
    {
        if(boundry[i].whether_object)
        {
            whether_any_object_detected = 1;
            distance_of_the_center_of_the_object_from_the_center_of_the_image = boundry[i].center.y-center_column_of_the_image;
            distance_of_the_center_of_the_object_from_the_center_of_the_image/=5;
            cout<<"Distance of the ball from the center of the screen is: "
                <<distance_of_the_center_of_the_object_from_the_center_of_the_image<<endl;
            if(distance_of_the_center_of_the_object_from_the_center_of_the_image>=0)
            {
                distance_of_the_center_of_the_object_from_the_center_of_the_image|=0x80;
                output_data_to_xbee[0]=distance_of_the_center_of_the_object_from_the_center_of_the_image;
                write_data_to_XBEE();
            }
            else
            {
                distance_of_the_center_of_the_object_from_the_center_of_the_image|=0xC0;
                output_data_to_xbee[0] = distance_of_the_center_of_the_object_from_the_center_of_the_image;
                write_data_to_XBEE();
            }
            break;
        }
    }
    if(!whether_any_object_detected)
    {
        output_data_to_xbee[0] = 0x00;
        write_data_to_XBEE();
    }

}

///Main Function
int main()
{

    VideoCapture camera(0);
    Mat original_image;
    if(!camera.isOpened())
    {
        cout<<"Can't open Camera!"<<endl;
        output_data_to_xbee[0]=0x02;
        write_data_to_XBEE();
        return -1;
    }

    namedWindow("original_image", CV_WINDOW_NORMAL); //Opens a window displaying the original image currently visible to the bot
    namedWindow("only_ballcolor", CV_WINDOW_NORMAL); //Opens a window displaying only those pixels from the original image whose color matches the color of the object to be detected
    namedWindow("only_boundries", CV_WINDOW_NORMAL); //Opens a window displaying only the boundaries of the detected blobs
    namedWindow("detected_object", CV_WINDOW_NORMAL); //Opens a window displaying the detected object
    if(!set_up_serial_connection())
    {
        cout<<"Error in setting up XBEEs"<<endl;
        return -1;
    }
    while(1)
    {
        read_data_from_XBEE();
        if(input_data_from_xbee[0] == 0x00)
        {
            if(!camera.read(original_image))
            {
                cout<<"Can't read frames from camera.";
                output_data_to_xbee[0]=0x02;
                write_data_to_XBEE();
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
            send_distance_from_center(original_image.cols/2);
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
}

