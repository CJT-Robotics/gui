#include <iostream>
#include <stdio.h>
#include <thread>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "stb_image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <cstdlib>
#include <array>
#include <string>
#include "std_msgs/String.h"
#include <std_msgs/Float32MultiArray.h>
#include <unistd.h>
#include <zbar.h>
#include <mutex>
#include <math.h>

#define STB_IMAGE_IMPLEMENTATION

using namespace ImGui;
using namespace ros;

const char *glsl_version = "#version 130";
int display_w, display_h;
int display_w_rostopics, display_h_rostopics;

const int targetWidth = 1280;
const int targetHeight = 720;

const int targetWidth_Rostopics = 650;
const int targetHeight_Rostopics = 900;

const int mainMenuBar = 23;
float mainMenuBarHeight;

bool darkMode = false;

bool shutdownBool = false;

int logo_width, logo_height, logo_channels;

int userPercentage = 0;
float robotVoltage = 0.f;
int ping = 1000;
float rpm[7] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

bool ret;
bool qr_active = false;
std::string qr_text;

bool displayRostopics = false;
int numTopics;

cv::Mat frameCameraTop;
cv::Mat oldFrameCameraTop;
bool isFrameCameraTop;
GLuint textureCameraTop;

GLuint my_image_texture_plusButton = 0;
GLuint my_image_texture_minusButton = 0;

int my_image_width_zoomButton = 32;
int my_image_height_zoomButton = 32;

GLuint my_image_texture_batteryEmpty = 0;
GLuint my_image_texture_batteryVeryLow = 0;
GLuint my_image_texture_batteryLow = 0;
GLuint my_image_texture_batteryMedium = 0;
GLuint my_image_texture_batteryOk = 0;
GLuint my_image_texture_batteryFine = 0;

GLuint my_image_texture_rpm = 0;

GLuint my_image_texture_signalstaerkeLow = 0;
GLuint my_image_texture_signalstaerkeMedium = 0;
GLuint my_image_texture_signalstaerkeFine = 0;

GLuint my_image_texture_Clock = 0;

int my_image_width_battery = 60;
int my_image_height_battery = 32;
int my_image_width_rpm = 50;
int my_image_height_rpm = 50;
int my_image_width_signalstaerke = 73;
int my_image_height_signalstaerke = 50;
int my_image_width_clock = 73;
int my_image_height_clock = 73;

int secondsUptime = 0;
int minutesUptime = 0;
int hoursUptime = 0;
int uptimeArray[3] = {0, 0, 0};

GLFWwindow *window;
GLFWwindow *rostopicsWindow;
ImFont *font;
ImVec4 background_color;

bool test = true;

std::string folderPath;
std::string ipv4_robot;
std::string rostopic_namespace;

zbar::ImageScanner scanner;

float disc_size = .01;
float disc_factor = 0.5 / disc_size;
float max_lidar_range = 3.5;

int lidar_image_size_x = 427;
int lidar_image_size_y = 360;

float lidar_robotWidth = 51;  // width of the Robot in cm
float lidar_robotLength = 58; // length of the Robot in cm
float lidarX = 25;            // the width-side distance of the lidar from the upper left corner
float lidarY = 40;            // the length-side distance from the Lidar of the upper left corner

bool lidar_connection_line_staus = true;
bool lidar_distance_line_status = false;
bool lidar_distnce_angle_staus = false;

float lidar_distance_angle_start = M_PI;
float lidar_distance_angle_fov = M_PI * 0.4f;

cv::Mat lidarImage = cv::Mat(lidar_image_size_y, lidar_image_size_x, CV_8UC3, cv::Scalar(255, 255, 255));
GLuint textureLidar;

std::mutex mutex;

void checkGLError()
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        std::cerr << "OpenGL error: " << err << std::endl;
    }
}

bool LoadTextureFromFile(const char *filename, GLuint *out_texture, int *out_width, int *out_height)
{
    // Load from file
    int image_width = 0;
    int image_height = 0;
    unsigned char *image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
    if (image_data == NULL)
        return false;

    // Create a OpenGL texture identifier
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

    // Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;

    return true;
}

void loadImagesGUI()
{

    std::string path = (folderPath + "src/imgs/").c_str();

    ret = LoadTextureFromFile((path + "zoom-plus.png").c_str(), &my_image_texture_plusButton, &my_image_width_zoomButton, &my_image_height_zoomButton);
    ret = LoadTextureFromFile((path + "zoom-minus.png").c_str(), &my_image_texture_minusButton, &my_image_width_zoomButton, &my_image_height_zoomButton);

    ret = LoadTextureFromFile((path + "batteryEmpty.png").c_str(), &my_image_texture_batteryEmpty, &my_image_width_battery, &my_image_height_battery);
    ret = LoadTextureFromFile((path + "batteryVeryLow.png").c_str(), &my_image_texture_batteryVeryLow, &my_image_width_battery, &my_image_height_battery);
    ret = LoadTextureFromFile((path + "batteryLow.png").c_str(), &my_image_texture_batteryLow, &my_image_width_battery, &my_image_height_battery);
    ret = LoadTextureFromFile((path + "batteryMedium.png").c_str(), &my_image_texture_batteryMedium, &my_image_width_battery, &my_image_height_battery);
    ret = LoadTextureFromFile((path + "batteryOk.png").c_str(), &my_image_texture_batteryOk, &my_image_width_battery, &my_image_height_battery);
    ret = LoadTextureFromFile((path + "batteryFine.png").c_str(), &my_image_texture_batteryFine, &my_image_width_battery, &my_image_height_battery);

    ret = LoadTextureFromFile((path + "rpm.png").c_str(), &my_image_texture_rpm, &my_image_width_rpm, &my_image_height_rpm);

    ret = LoadTextureFromFile((path + "signalstaerkeLow.png").c_str(), &my_image_texture_signalstaerkeLow, &my_image_width_signalstaerke, &my_image_height_signalstaerke);
    ret = LoadTextureFromFile((path + "signalstaerkeMedium.png").c_str(), &my_image_texture_signalstaerkeMedium, &my_image_width_signalstaerke, &my_image_height_signalstaerke);
    ret = LoadTextureFromFile((path + "signalstaerkeFine.png").c_str(), &my_image_texture_signalstaerkeFine, &my_image_width_signalstaerke, &my_image_height_signalstaerke);

    ret = LoadTextureFromFile((path + "clock.png").c_str(), &my_image_texture_Clock, &my_image_width_clock, &my_image_height_clock);

    IM_ASSERT(ret);
}

void getPing()
{

    while (true && !shutdownBool)
    {
        // Ausführen des ping-Befehls und Auslesen der Ausgabe
        std::string pingCommand = "ping -c 1 " + ipv4_robot;
        FILE *pingPipe = popen(pingCommand.c_str(), "r");
        if (!pingPipe)
        {
            std::cerr << "Error executing ping command." << std::endl;
        }

        // Analyse der ping-Antwort
        const int bufferSize = 128;
        std::array<char, bufferSize> buffer;
        while (!feof(pingPipe))
        {
            if (fgets(buffer.data(), bufferSize, pingPipe) != nullptr)
            {
                std::string bufferStr(buffer.data());
                // Hier kannst du die Ausgabe analysieren und die Latenz extrahieren
                // Zum Beispiel: "time=XX.XX ms"
                size_t pos = bufferStr.find("Zeit=");
                if (pos != std::string::npos)
                {
                    pos += 5; // Überspringe "time="
                    ping = std::stoi(bufferStr.substr(pos));
                }
            }
        }

        // Schließen des Pipe und Warten
        pclose(pingPipe);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int getUserPercentage(NodeHandle nh)
{

    std::string path;
    std::string tempVar;
    nh.getParam("laptop_battery", tempVar);
    path = ("/sys/class/power_supply/BAT0/subsystem/BAT0/" + tempVar + "_");

    FILE *battChargeNow;
    FILE *battChargeFull;
    long unsigned int battMax_mAh = 0;
    long unsigned int battRemain_mAh = 0;

    while (true && !shutdownBool)
    {

        if (NULL == (battChargeNow = fopen((path + "now").c_str(), "r")))
        {
            fclose(battChargeNow);
            return -1;
        }
        if (NULL == (battChargeFull = fopen((path + "full").c_str(), "r")))
        {
            fclose(battChargeNow);
            fclose(battChargeFull);
            return -1;
        }

        fscanf((FILE *)battChargeFull, "%lu", &battMax_mAh);
        fscanf((FILE *)battChargeNow, "%lu", &battRemain_mAh);

        userPercentage = 100.00 * ((float)battRemain_mAh / (float)battMax_mAh);

        std::this_thread::sleep_for(std::chrono::seconds(60));
    }

    return 0;
}

void getRobotVoltage(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    robotVoltage = msg->data[0];
}
void getUptime()
{

    while (true && !shutdownBool)
    {

        secondsUptime++;

        if (secondsUptime == 60)
        {

            secondsUptime = 0;
            minutesUptime++;

            if (minutesUptime == 60)
            {
                minutesUptime = 0;
                hoursUptime++;
            }
        }

        uptimeArray[2] = secondsUptime;
        uptimeArray[1] = minutesUptime;
        uptimeArray[0] = hoursUptime;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void getRPM(const std_msgs::Float32MultiArray::ConstPtr &msg)
{

    for (int w = 1; w < 7; w++)
    {
        rpm[w] = msg->data[w];
        if (rpm[w] == -0.0f)
        {
            rpm[w] = 0.0f;
        }
    }

    getRobotVoltage(msg);
}

// Temporär

void crash(){
    int array[1] = {0};
    array[2] = 1;
}

void writeTopics(std::string rostopicText[])
{
    for (int i = 0; i < numTopics; i++)
    {
        std::cout << rostopicText[i] << std::endl;
    }
}

void getRostopics()
{

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    numTopics = master_topics.size();
    std::string rostopicText[numTopics];

    std::cout << "Number of ROS Topics: " << numTopics << std::endl;

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    {

        const ros::master::TopicInfo &info = *it;
        rostopicText[it - master_topics.begin()] = info.name;
    }

    // Temporär
    writeTopics(rostopicText);
}

// void getQrCode(cv::Mat frameCameraTop) {

//     std::string path = (folderPath + "src/imgs/").c_str();

//     cv::Mat testMat = cv::imread((path + "test.png").c_str());

//     while (true && !shutdownBool) {
//         zbar::Image zbar_image(testMat.cols, testMat.rows, "Y800", testMat.data, testMat.cols * testMat.rows);
//         int n = scanner.scan(zbar_image);

//         for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
//         {
//             std::string qr_data = symbol->get_data();
//             std::cout<<qr_data.c_str()<<std::endl;
//             std::cout<<"nein"<<std::endl;
//         }

//         zbar_image.set_data(NULL, 0);

//         std::cout<<"Test!"<<std::endl;
//         std::this_thread::sleep_for(std::chrono::seconds(5));
//     }

// }

void getTopCamera(const sensor_msgs::ImageConstPtr &msg)
{

    try
    {

        frameCameraTop = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(frameCameraTop, frameCameraTop, cv::COLOR_BGR2RGBA);

        cv::Point2f center(frameCameraTop.cols / 2, frameCameraTop.rows / 2);
        double angle = 180.0;
        double scale = 0.8;

        cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, scale);

        cv::Mat rotatedMat;
        cv::warpAffine(frameCameraTop, rotatedMat, rotationMatrix, frameCameraTop.size());

        frameCameraTop = rotatedMat;
        oldFrameCameraTop = frameCameraTop;

        isFrameCameraTop = true;
    }
    catch (cv_bridge::Exception &e)
    {
        isFrameCameraTop = false;
    }
}

void displayTopCamera(const sensor_msgs::ImageConstPtr &msg)
{
}

void laserCallback(const sensor_msgs::LaserScan &msg)
{
    float maxAngle = msg.angle_max;
    float minAngle = msg.angle_min;
    float angleInc = msg.angle_increment;
    float maxLength = msg.range_max;

    float angle;

    lidarImage = cv::Mat(lidar_image_size_y, lidar_image_size_x, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<float> ranges = msg.ranges;
    int num_pts = ranges.size();

    cv::Point image_mid = cv::Point(lidar_image_size_x / 2, lidar_image_size_y / 2);

    float xy_scan[num_pts][2] = {};

    for (int i = 0; i < num_pts; i++)
    {
        if (ranges[i] > 10 || ranges[i] == NULL)
            ;
        else
        {
            // calculates the angle of the point and the x and y position
            angle = minAngle + i * angleInc;
            xy_scan[i][0] = ranges[i] * cos(angle);
            xy_scan[i][1] = ranges[i] * sin(angle);
        }
    }

    for (int i = 0; i < num_pts; i++)
    {
        float pt_x = xy_scan[i][1];
        float pt_y = xy_scan[i][0];

        if (pt_x < max_lidar_range || pt_x > -1 * (max_lidar_range - disc_size) || pt_y < max_lidar_range || pt_y > -1 * (max_lidar_range - disc_size))
        {
            int pix_x = (int)(lidar_image_size_x / 2 - pt_x * disc_factor);
            int pix_y = (int)(lidar_image_size_y / 2 - pt_y * disc_factor);

            if (i < (num_pts - 1) && lidar_connection_line_staus)
            { // connects two points with a line
                int pix_x_new = (int)(lidar_image_size_x / 2 - xy_scan[i + 1][1] * disc_factor);
                int pix_y_new = (int)(lidar_image_size_y / 2 - xy_scan[i + 1][0] * disc_factor);

                cv::Point old_pos = cv::Point(pix_x, pix_y);
                cv::Point new_pos = cv::Point(pix_x_new, pix_y_new);

                float dist = cv::norm(old_pos - new_pos);

                if (dist < 15){
                    //mutex.lock();
                    cv::line(lidarImage, old_pos, new_pos, cv::Scalar(0, 0, 0), 1);
                    //mutex.unlock();
                }
            }

            if (pix_x < lidar_image_size_x && pix_y < lidar_image_size_y && pix_x > 0 && pix_y > 0) {
                //mutex.lock();
                lidarImage.at<cv::Vec3b>(cv::Point(pix_x, pix_y)) = cv::Vec3b(0, 0, 0);
                //mutex.unlock();
            }
        }
    }

    int pixCount = 0;
    float prerange = 0.f;
    int startIndex = 0;
    std::vector<cv::Point> endDistanceLine = std::vector<cv::Point>(0);
    std::vector<int> distanceIndex = std::vector<int>(0);
    int objectCount = 0;
    int distance_line_i;

    //draws the distance lines
    if(lidar_distance_line_status)  {
        //calculates the distance lines
        for(distance_line_i = 0; distance_line_i < num_pts; distance_line_i++)    {
            if(abs(prerange - ranges[distance_line_i]) < 0.15 && ranges[distance_line_i] < 10)  {
                pixCount ++;
            } else if(abs(prerange - ranges[distance_line_i]) > 0.15 && pixCount > 50) {
                objectCount ++;
                int middleIndex = (int)(startIndex + 0.5 * abs(startIndex - (distance_line_i-1)));

                int pt_x_mid = (int)(lidar_image_size_x/2 - xy_scan[middleIndex][1] * disc_factor);
                int pt_y_mid = (int)(lidar_image_size_y/2 - xy_scan[middleIndex][0] * disc_factor);

                endDistanceLine.insert(endDistanceLine.end(),cv::Point(pt_x_mid,pt_y_mid));

                distanceIndex.insert(distanceIndex.end(),middleIndex);
                pixCount = 0;
            }

            if(abs(prerange - ranges[distance_line_i]) > 0.15 && ranges[distance_line_i] < 10)
                startIndex = distance_line_i;

            prerange = ranges[distance_line_i];
        }
        if(ranges[num_pts-1] < 10)  {
            objectCount ++;
            int middleIndex = (int)(startIndex + 0.5 * abs(startIndex - (distance_line_i-1)));

            int pt_x_mid = (int)(lidar_image_size_x/2 - xy_scan[middleIndex][1] * disc_factor);
            int pt_y_mid = (int)(lidar_image_size_y/2 - xy_scan[middleIndex][0] * disc_factor);

            endDistanceLine.insert(endDistanceLine.end(),cv::Point(pt_x_mid,pt_y_mid));

            distanceIndex.insert(distanceIndex.end(),middleIndex);
            pixCount = 0;
        }

        //mutex.lock();
        //draws the distanceLines
        for(int i = 0; i < endDistanceLine.size(); i++)   {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << ranges[distanceIndex[i]];
            std::string objectDistance = ss.str();

            cv::line(lidarImage,cv::Point((int)(lidar_image_size_x / 2),(int)(lidar_image_size_y / 2)),endDistanceLine[i], cv::Scalar(0, 0, 0),1);  
            cv::putText(lidarImage,objectDistance,cv::Point(endDistanceLine[i]),cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,0,0),1,true);
        }
        //mutex.unlock();
    }

    //draws the distance angle
    if(lidar_distnce_angle_staus)   {
        //calculates the distance angle
        int angleStartIndex = (int)((minAngle + lidar_distance_angle_fov) / angleInc);
        int angleEndIndex = (int)(angleStartIndex + lidar_distance_angle_fov / angleInc);

        bool angleFound = false;

        if(ranges[angleEndIndex] > max_lidar_range || ranges[angleEndIndex] == NULL || ranges[angleEndIndex < 0.1]) {
            while (!angleFound)
            {
                angleEndIndex ++;
                if(ranges[angleEndIndex] > max_lidar_range || ranges[angleEndIndex] == NULL || ranges[angleEndIndex < 0.1]) {
                    angleFound = true;
                }
            }
        }

        if(ranges[angleStartIndex] > max_lidar_range || ranges[angleStartIndex] == NULL || ranges[angleStartIndex < 0.1]) {
            angleFound = false; 
            while (!angleFound)
            {
                angleStartIndex ++;
                if(ranges[angleStartIndex] > max_lidar_range || ranges[angleStartIndex] == NULL || ranges[angleStartIndex < 0.1]) {
                    angleFound = true;
                }
            }
        }



        //mutex.lock();
        //draws the distance angle

        cv::point lineStart = cv::Point((int)(lidar_image_size_x/2 - xy_scan[angleStartIndex][1] * disc_factor),(int)(lidar_image_size_y/2 - xy_scan[angleStartIndex][0] * disc_factor))
        cv::point lineEnd = cv::Point((int)(lidar_image_size_x/2 - xy_scan[angleEndIndex][1] * disc_factor),(int)(lidar_image_size_y/2 - xy_scan[angleEndIndex][0] * disc_factor))
        
        cv::line(lidarImage, image_mid, lineStart, cv::Scalar(0, 0, 0),1);  //line to start of angle
        cv::line(lidarImage, image_mid, lineEnd, cv::Scalar(0, 0, 0),1);    //line to end og angle

        //draws values in the distane Angle
        for(int i = angleStartIndex; i < angleEndIndex; i += 20)    {
            int angle_pix_x = (int)(lidar_image_size_x/2 - xy_scan[i][1] * disc_factor);
            int angle_pix_y = (int)(lidar_image_size_y/2 - xy_scan[i][0] * disc_factor);

            std::stringstream ss2;
            ss << std::fixed << std::setprecision(2) << ranges[i];
            std::string objectDistance2 = ss2.str();

            if(angle_pix_x != lidar_image_size_x || angle_pix_y != lidar_image_size_y)
                cv::putText(lidarImage, objectDistance, cv::point(angle_pix_x, angle_pix_y), cv::FONT_HERSHEY_DUPLEX,0.5,cv::Scalar(0,0,0),1,true);

        }

        //mutex.unlock();
        
        }


    //draws the rectangle that represents the robots size
    int x_pos_rect = (int)(lidar_image_size_x/2 - ((lidarX / 100) * disc_factor));
    int y_pos_rect = (int)(lidar_image_size_y/2 - ((lidarY / 100) * disc_factor));
    int rect_width = (int)((lidar_robotWidth/100)*disc_factor);
    int rect_height = (int)((lidar_robotLength/100) * disc_factor);

    //mutex.lock();
    for(int x = x_pos_rect; x < x_pos_rect + rect_width; x++)
    for(int y = y_pos_rect; y < y_pos_rect + rect_height; y++){
        lidarImage.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(120,120,120);
        }

    for(int x = (int)(lidar_image_size_x/2 - 2); x < lidar_image_size_x/2+2; x++)
    for(int y = (int)(lidar_image_size_y/2 - 2); y < lidar_image_size_y/2+2; y++){
        lidarImage.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,100,0);
    }
    //mutex.unlock();

    if(lidar_distance_line_status || lidar_distnce_angle_staus)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void setMainMenuBar()
{

    ImVec4 newHoverColor;
    ImVec4 newBgColor;

    ImGuiStyle &style = ImGui::GetStyle();

    if (!darkMode)
    {
        newHoverColor = ImVec4(0.65f, 0.65f, 0.65f, 1.0f);
        newBgColor = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);

        style.Colors[ImGuiCol_HeaderHovered] = newHoverColor;
        style.Colors[ImGuiCol_Header] = newHoverColor;
        style.Colors[ImGuiCol_MenuBarBg] = newBgColor;
        style.Colors[ImGuiCol_PopupBg] = newBgColor;
    }
    else
    {
        newHoverColor = ImVec4(0.51f, 0.51f, 0.49f, 1.0f);
        newBgColor = ImVec4(0.31f, 0.31f, 0.29f, 1.0f);

        style.Colors[ImGuiCol_HeaderHovered] = newHoverColor;
        style.Colors[ImGuiCol_Header] = newHoverColor;
        style.Colors[ImGuiCol_MenuBarBg] = newBgColor;
        style.Colors[ImGuiCol_PopupBg] = newBgColor;
    }

    ImGui::BeginMainMenuBar();

    if (ImGui::BeginMenu("Ansicht"))
    {
        if (ImGui::MenuItem("Light Mode"))
        {
            darkMode = false;
        }
        if (ImGui::MenuItem("Dark Mode"))
        {
            darkMode = true;
        }
        if (ImGui::MenuItem("Vistitor Modus"))
        {
            /* code */
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Optionen"))
    {
        if (ImGui::BeginMenu("Einblenden"))
        {
            ImGui::MenuItem("Camera Front");
            ImGui::MenuItem("Camera Back");
            ImGui::MenuItem("LIDAR Temp");
            ImGui::MenuItem("Camera Temp1");
            ImGui::MenuItem("Camera Temp2");
            ImGui::MenuItem("Camera Temp3");
            ImGui::MenuItem("Camera Temp4");
            ImGui::MenuItem("Akku Laptop");
            ImGui::MenuItem("Spannung Robot");
            ImGui::MenuItem("RPM");
            ImGui::MenuItem("Ping");
            ImGui::MenuItem("Uptime");
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Ausblenden"))
        {
            ImGui::MenuItem("Camera Front");
            ImGui::MenuItem("Camera Back");
            ImGui::MenuItem("LIDAR Temp");
            ImGui::MenuItem("Camera Temp1");
            ImGui::MenuItem("Camera Temp2");
            ImGui::MenuItem("Camera Temp3");
            ImGui::MenuItem("Camera Temp4");
            ImGui::MenuItem("Akku Laptop");
            ImGui::MenuItem("Spannung Robot");
            ImGui::MenuItem("RPM");
            ImGui::MenuItem("Ping");
            ImGui::MenuItem("Uptime");
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem("QR-Code Scanner"))
        {
            qr_active = !qr_active;
        }
        if (ImGui::MenuItem("Hilfslinien Räder"))
        {
        }

        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Extras"))
    {
        if (ImGui::MenuItem("rostopics auflisten"))
        {
            displayRostopics = !displayRostopics;

            if (displayRostopics)
            {
                getRostopics();
            }
        }
        if (ImGui::MenuItem("Chat öffnen"))
        {
            test = true;
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Hilfe"))
    {
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

void infillTopLeftWindow()
{

    float windowSizeX = ImGui::GetWindowSize().x;
    float windowSizeY = ImGui::GetWindowSize().y;

    ImGui::SetCursorPos(ImVec2(5.f, windowSizeY - 20.f));
    ImGui::Text("Kamera vorne");
}
void infillTopRightWindow()
{

    float windowSizeX = ImGui::GetWindowSize().x;
    float windowSizeY = ImGui::GetWindowSize().y;

    ImGui::SetCursorPos(ImVec2(5.f, windowSizeY - 20.f));
    ImGui::Text("Kamera hinten");
}
void infillBottomLeftWindow()
{
    //mutex.lock();
    cv::Mat lidarImageCopy;
    lidarImage.copyTo(lidarImageCopy);
    //mutex.unlock();

    glGenTextures(1, &textureLidar);
    glBindTexture(GL_TEXTURE_2D, textureLidar);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    ImVec2 windowSize = ImGui::GetWindowSize();
    float windowSizeX = ImGui::GetWindowSize().x;
    float windowSizeY = ImGui::GetWindowSize().y;

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0, 0, 0, 0));

    ImGui::SetCursorPos(ImVec2(0.f, 0.f));

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, lidarImageCopy.cols, lidarImageCopy.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, lidarImageCopy.data);
    ImGui::SetCursorPos(ImVec2(0.f, 0.f));
    ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(textureLidar)), ImVec2(lidarImageCopy.cols, lidarImageCopy.rows));

    ImGui::SetCursorPos(ImVec2(windowSizeX - 45.f, ((windowSizeY / 2.f) - 8.f) - 10.f));
    if (ImGui::ImageButton((void *)(intptr_t)my_image_texture_plusButton, ImVec2(32.f, 32.f)))
    {
        if (disc_factor < 100)
            disc_factor += 5;
    }

    ImGui::SetCursorPos(ImVec2(windowSizeX - 45.f, ((windowSizeY / 2.f) + 8.f) + 10.f));
    if (ImGui::ImageButton((void *)(intptr_t)my_image_texture_minusButton, ImVec2(32.f, 32.f)))
    {
        if (disc_factor > 20)
            disc_factor -= 5;
    }

    ImGui::PopStyleColor(3);

    ImGui::SetCursorPos(ImVec2(5.f, windowSizeY - 20.f));
    ImGui::Text("LIDAR");
}
void infillBottomMiddleWindow()
{

    glGenTextures(1, &textureCameraTop);
    glBindTexture(GL_TEXTURE_2D, textureCameraTop);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

    if (isFrameCameraTop)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, frameCameraTop.cols, frameCameraTop.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, frameCameraTop.data);
        ImGui::SetCursorPos(ImVec2(-60.f, -60.f));
        ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(textureCameraTop)), ImVec2(frameCameraTop.cols, frameCameraTop.rows));

        isFrameCameraTop = false;
    }
    else
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, oldFrameCameraTop.cols, oldFrameCameraTop.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, oldFrameCameraTop.data);
        ImGui::SetCursorPos(ImVec2(-60.f, -60.f));
        ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(textureCameraTop)), ImVec2(oldFrameCameraTop.cols, oldFrameCameraTop.rows));
    }

    float windowSizeX = ImGui::GetWindowSize().x;
    float windowSizeY = ImGui::GetWindowSize().y;

    ImGui::SetCursorPos(ImVec2(5.f, windowSizeY - 20.f));
    ImGui::Text("Kamera temporär");
}
void infillBottomRightWindow()
{

    ImVec2 windowSize = ImGui::GetWindowSize();
    ImVec2 textSize = ImGui::CalcTextSize("Wichitge Infos");
    float centerX = (windowSize.x - textSize.x) * 0.5f;
    float windowSizeX = ImGui::GetWindowSize().x;
    float windowSizeY = ImGui::GetWindowSize().y;

    std::string rpmString;

    ImGui::SetCursorPosX(centerX);
    ImGui::Text("Wichitge Infos");

    ImGui::SetCursorPos(ImVec2(10.f, 35.f));
    if (userPercentage > 90.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryFine, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (userPercentage > 70.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryOk, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (userPercentage > 30.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryMedium, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (userPercentage > 20.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryLow, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (userPercentage > 0.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryVeryLow, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryEmpty, ImVec2(my_image_width_battery, my_image_height_battery));
    }

    std::string userPercentageString = std::to_string(userPercentage);
    userPercentageString.append("%");
    ImGui::SetCursorPos(ImVec2(80.f, 43.f));
    ImGui::Text("%s", userPercentageString.c_str());

    ImGui::SetCursorPos(ImVec2(10.f, 70.f));
    if (robotVoltage > 28.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryFine, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (robotVoltage > 26.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryOk, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (robotVoltage > 25.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryMedium, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (robotVoltage > 23.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryLow, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else if (robotVoltage > 22.f)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryVeryLow, ImVec2(my_image_width_battery, my_image_height_battery));
    }
    else
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_batteryEmpty, ImVec2(my_image_width_battery, my_image_height_battery));
    }

    std::string formattedVoltageString = std::to_string(robotVoltage);
    size_t dotPosition = formattedVoltageString.find('.');
    if (dotPosition != std::string::npos && dotPosition + 3 < formattedVoltageString.length())
    {
        formattedVoltageString.erase(dotPosition + 3);
    }
    formattedVoltageString.append("V");

    ImGui::SetCursorPos(ImVec2(80.f, 78.f));
    ImGui::Text("%s", formattedVoltageString.c_str());

    ImGui::SetCursorPos(ImVec2(10.f, 115.f));
    ImGui::Image((void *)(intptr_t)my_image_texture_rpm, ImVec2(my_image_width_rpm, my_image_height_rpm));
    rpmString = std::to_string(rpm[2]);
    dotPosition = rpmString.find('.');
    if (dotPosition != std::string::npos && dotPosition + 2 < rpmString.length())
    {
        rpmString.erase(dotPosition + 2);
    }
    ImGui::SetCursorPos(ImVec2(25.f, 130.f));
    ImGui::Text("%s", rpmString.c_str());

    ImGui::SetCursorPos(ImVec2(70.f, 115.f));
    ImGui::Image((void *)(intptr_t)my_image_texture_rpm, ImVec2(my_image_width_rpm, my_image_height_rpm));
    rpmString = std::to_string(rpm[4]);
    dotPosition = rpmString.find('.');
    if (dotPosition != std::string::npos && dotPosition + 2 < rpmString.length())
    {
        rpmString.erase(dotPosition + 2);
    }
    ImGui::SetCursorPos(ImVec2(85.f, 130.f));
    ImGui::Text("%s", rpmString.c_str());

    ImGui::SetCursorPos(ImVec2(130.f, 115.f));
    ImGui::Image((void *)(intptr_t)my_image_texture_rpm, ImVec2(my_image_width_rpm, my_image_height_rpm));
    rpmString = std::to_string(rpm[6]);
    dotPosition = rpmString.find('.');
    if (dotPosition != std::string::npos && dotPosition + 2 < rpmString.length())
    {
        rpmString.erase(dotPosition + 2);
    }
    ImGui::SetCursorPos(ImVec2(145.f, 130.f));
    ImGui::Text("%s", rpmString.c_str());

    ImGui::SetCursorPos(ImVec2(10.f, 168.f));
    ImGui::Image((void *)(intptr_t)my_image_texture_rpm, ImVec2(my_image_width_rpm, my_image_height_rpm));
    rpmString = std::to_string(rpm[1]);
    dotPosition = rpmString.find('.');
    if (dotPosition != std::string::npos && dotPosition + 2 < rpmString.length())
    {
        rpmString.erase(dotPosition + 2);
    }
    ImGui::SetCursorPos(ImVec2(25.f, 183.f));
    ImGui::Text("%s", rpmString.c_str());

    ImGui::SetCursorPos(ImVec2(70.f, 168.f));
    ImGui::Image((void *)(intptr_t)my_image_texture_rpm, ImVec2(my_image_width_rpm, my_image_height_rpm));
    rpmString = std::to_string(rpm[3]);
    dotPosition = rpmString.find('.');
    if (dotPosition != std::string::npos && dotPosition + 2 < rpmString.length())
    {
        rpmString.erase(dotPosition + 2);
    }
    ImGui::SetCursorPos(ImVec2(85.f, 183.f));
    ImGui::Text("%s", rpmString.c_str());

    ImGui::SetCursorPos(ImVec2(130.f, 168.f));
    ImGui::Image((void *)(intptr_t)my_image_texture_rpm, ImVec2(my_image_width_rpm, my_image_height_rpm));
    rpmString = std::to_string(rpm[5]);
    dotPosition = rpmString.find('.');
    if (dotPosition != std::string::npos && dotPosition + 2 < rpmString.length())
    {
        rpmString.erase(dotPosition + 2);
    }
    ImGui::SetCursorPos(ImVec2(145.f, 183.f));
    ImGui::Text("%s", rpmString.c_str());

    ImGui::SetCursorPos(ImVec2(10.f, 225.f));
    if (ping < 25)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_signalstaerkeFine, ImVec2(my_image_width_signalstaerke, my_image_height_signalstaerke));
    }
    else if (ping < 75)
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_signalstaerkeMedium, ImVec2(my_image_width_signalstaerke, my_image_height_signalstaerke));
    }
    else
    {
        ImGui::Image((void *)(intptr_t)my_image_texture_signalstaerkeLow, ImVec2(my_image_width_signalstaerke, my_image_height_signalstaerke));
    }

    ImGui::SetCursorPos(ImVec2(100.f, 245.f));
    std::string pingString = std::to_string(ping);
    pingString.append("ms");
    ImGui::Text("%s", pingString.c_str());

    ImGui::SetCursorPos(ImVec2(10.f, 280.f));
    ImGui::Image((void *)(intptr_t)my_image_texture_Clock, ImVec2(my_image_width_clock, my_image_height_clock));

    std::string uptimeString = "";
    if (uptimeArray[0] < 10)
    {
        uptimeString += "0" + std::to_string(uptimeArray[0]);
    }
    else
    {
        uptimeString += std::to_string(uptimeArray[0]);
    }
    uptimeString += ":";
    if (uptimeArray[1] < 10)
    {
        uptimeString += "0" + std::to_string(uptimeArray[1]);
    }
    else
    {
        uptimeString += std::to_string(uptimeArray[1]);
    }
    uptimeString += ":";
    if (uptimeArray[2] < 10)
    {
        uptimeString += "0" + std::to_string(uptimeArray[2]);
    }
    else
    {
        uptimeString += std::to_string(uptimeArray[2]);
    }

    ImGui::SetCursorPos(ImVec2(100.f, 307.f));
    ImGui::Text("%s", uptimeString.c_str());
}

void setDividingLines()
{

    ImVec2 topRowPos(0, 0);
    ImVec2 topLeftSize(display_w * 0.5f, display_h * 0.5f);
    ImVec2 topRightSize(display_w * 0.5f, display_h * 0.5f);
    ImVec2 bottomRowPos(0, display_h * 0.5f);
    ImVec2 bottomLeftSize(display_w * 0.5f * (2.f / 3.f), display_h * 0.5f);
    ImVec2 bottomMiddleSize(display_w * 0.5f, display_h * 0.5f);
    ImVec2 bottomRightSize(display_w * 0.5f * (1.f / 3.f), display_h * 0.5f);

    ImGui::SetNextWindowPos(topRowPos);
    ImGui::SetNextWindowSize(topLeftSize);
    ImGui::Begin("Top Left", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    infillTopLeftWindow();
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(display_w * 0.5f, 0));
    ImGui::SetNextWindowSize(topRightSize);
    ImGui::Begin("Top Right", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    infillTopRightWindow();
    ImGui::End();

    ImGui::SetNextWindowPos(bottomRowPos);
    ImGui::SetNextWindowSize(bottomLeftSize);
    ImGui::Begin("Bottom Left", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    infillBottomLeftWindow();
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0 + display_w * 0.5f * (2.f / 3.f), display_h * 0.5f));
    ImGui::SetNextWindowSize(bottomMiddleSize);
    ImGui::Begin("Bottom Middle", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    infillBottomMiddleWindow();
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0 + display_w * 0.5f * (2.f / 3.f) + display_w * 0.5f, display_h * 0.5f));
    ImGui::SetNextWindowSize(bottomRightSize);
    ImGui::Begin("Bottom Right", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    infillBottomRightWindow();
    ImGui::End();
}

int gui()
{
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(targetWidth, targetHeight, "Driver_GUI — CJT-Robotics — Home", NULL, NULL);
    if (!window)
    {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }

    std::string path = (folderPath + "src/imgs/").c_str();
    unsigned char *logo_image = stbi_load((path + "icon.png").c_str(), &logo_width, &logo_height, &logo_channels, STBI_rgb_alpha);

    GLFWimage icon;
    icon.width = logo_width;
    icon.height = logo_height;
    icon.pixels = logo_image;

    glfwSetWindowAspectRatio(window, targetWidth, targetHeight);
    glfwMakeContextCurrent(window);
    glfwSetWindowIcon(window, 1, &icon);
    stbi_image_free(logo_image);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls

    font = io.Fonts->AddFontFromFileTTF((folderPath + "src/Roboto-Light.ttf").c_str(), 18.0f);

    ImGui_ImplGlfw_InitForOpenGL(window, true); // Setup Platform/Renderer backends
    ImGui_ImplOpenGL3_Init(glsl_version);       // Setup Platform/Renderer backends


    loadImagesGUI();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiStyle &style = ImGui::GetStyle();

        if (!darkMode)
        {
            background_color = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);
            style.Colors[ImGuiCol_WindowBg] = ImVec4(0.9f, 0.9f, 0.9f, 1.0f);
            style.Colors[ImGuiCol_Text] = ImVec4(0.10f, 0.10f, 0.10f, 1.0f);
        }
        else
        {
            background_color = ImVec4(0.31f, 0.31f, 0.29f, 1.0f);
            style.Colors[ImGuiCol_WindowBg] = ImVec4(0.31f, 0.31f, 0.29f, 1.0f);
            style.Colors[ImGuiCol_Text] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
        }

        setMainMenuBar();

        setDividingLines();

        ImGui::Render();
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(background_color.x * background_color.w, background_color.y * background_color.w, background_color.z * background_color.w, background_color.w);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    shutdownBool = true;

    crash();
    return 0;

}

void loadLidar(NodeHandle nh)
{

    ros::Subscriber laserSub = nh.subscribe("/cjt/scan", 1, laserCallback);

    while (ros::ok())
    {

        ros::spinOnce();

        if (shutdownBool)
        {
            ros::shutdown();
        }
    }
}

void loadCamera(NodeHandle nh, image_transport::ImageTransport it) {

    std::string tempVar;
    std::string path;

    nh.getParam("rostopic_name_cameraTop", tempVar);
    path = nh.getNamespace() + tempVar;

    image_transport::Subscriber imsub = it.subscribe(path.c_str(), 1, getTopCamera, image_transport::TransportHints("compressed"));
    //imsub = it.subscribe(path.c_str(), 1, missing, image_transport::TransportHints("compressed"));

    while (ros::ok())
    {

        ros::spinOnce();

        if (shutdownBool)
        {
            ros::shutdown();
        }
    }

}

void loadSubscriberRpm(NodeHandle nh)
{

    std::string tempVar;
    std::string path;

    nh.getParam("rostopic_name_rpm", tempVar);
    path = nh.getNamespace() + tempVar;

    Subscriber rpmSubscriber = nh.subscribe(path.c_str(), 1, getRPM);
    
    while (ros::ok())
    {

        ros::spinOnce();

        if (shutdownBool)
        {
            ros::shutdown();
        }
    }
    
}

void loadFromLaunchFile(NodeHandle nh)
{

    std::string tempVar;

    nh.getParam("path_to_catkin_folder", tempVar);
    folderPath = tempVar + "/src/";
    nh.getParam("folder_name", tempVar);
    folderPath = folderPath + tempVar + "/";

    nh.getParam("ipv4_address_robot", tempVar);
    ipv4_robot = tempVar;

    nh.getParam("rostopic_namespace", tempVar);
    rostopic_namespace = tempVar;
}

void threadLoader(NodeHandle nh, image_transport::ImageTransport it)
{

    std::thread rpmSubscriberThread(loadSubscriberRpm, nh);
    std::thread cameraSubscriberThread(loadCamera, nh, it);
    std::thread lidarThread(loadLidar, nh);
    std::thread percentage(getUserPercentage, nh);
    std::thread pingen(getPing);
    std::thread time(getUptime);
    // std::thread qrCode(getQrCode, frameCameraTop);
    std::thread setGui(gui);

    rpmSubscriberThread.join();
    cameraSubscriberThread.join();
    lidarThread.join();
    percentage.join();
    pingen.join();
    time.join();
    // qrCode.join();
    setGui.join();
    
}

int main(int argc, char *argv[])
{

    init(argc, argv, "cjtrobotics_gui_driver");

    NodeHandle nh;
    image_transport::ImageTransport it(nh);

    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    loadFromLaunchFile(nh);

    threadLoader(nh, it);

    return 0;
}
