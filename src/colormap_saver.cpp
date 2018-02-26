// made by Yuki Katsumata 2018.1.15

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

#include <sstream>

using namespace std;

int COLOR[211][3] = {{255,255,255},
{204,102,204}, {  0,255,204}, {255,153,204}, {  0,153, 51}, {255,  0,153}, {255,255,204}, {204,153,153}, { 51,255,  0}, { 51,102, 51}, {  0, 51,  0},
{102, 51,102}, {102,102,153}, {255, 51,153}, {255,102,204}, {  0,102,153}, {102,102,  0}, {255,204,255}, { 51,204,153}, {204,102,102}, { 51,204,255},
{153,255,102}, {204,153, 51}, {  0,255,102}, {  0,153,255}, {153,204,102}, {102,204,255}, {102,255,153}, { 51,153, 51}, {102,  0,  0}, { 51,102,  0},
{  0, 51,102}, {255, 51,102}, {204,153,  0}, { 51, 51,255}, {153,102, 51}, {102,102, 51}, {153,153,  0}, { 51, 51,204}, { 51,153,102}, {102,153,153},
{204,102,  0}, {153,204,153}, {  0,  0,204}, {  0,204,102}, {255,204,204}, { 51,  0,255}, { 51,153,255}, {153,  0,255}, {255,  0,255}, {255,102,255},
{204, 51,204}, {  0,204, 51}, {204,102,255}, {255,153, 51}, {255,255,102}, {153,255,  0}, {153,255,255}, {255,255,153}, {  0,102,102}, {255,  0,204},
{204, 51, 51}, {  0,  0,255}, {  0,204,204}, {204,255,102}, {102, 51, 51}, {153,204,  0}, {102,204,204}, { 51,204,102}, {153,204,204}, {102,255,102},
{102,153,  0}, {204,255,153}, {  0,153,204}, {153,153,102}, {102,  0,255}, {153, 51,204}, { 51,204,204}, {102,255,204}, {153,102,102}, {255,102,102},
{255,  0,  0}, {255, 51, 51}, {204,102,153}, {153,102,255}, {102,255,  0}, { 51,153,204}, {153, 51,255}, {204, 51,255}, {102,  0,102}, {  0,102,  0},
{204,204,102}, { 51, 51,153}, {  0,255,255}, { 51,  0,  0}, {153,153,204}, {204,  0,  0}, {  0, 51, 51}, {  0, 51,153}, {153,  0,  0}, {  0,102,255},
{  0,255,153}, {204,204,153}, {  0,  0, 51}, {102, 51,  0}, {153,102,153}, { 51,102,204}, {102,153,204}, { 51,255,204}, {102, 51,255}, {204,255,204},
{255,255, 51}, { 51,  0,102}, {255,255,  0}, {255,153,255}, {153,153, 51}, {153, 51,153}, {  0,  0,102}, { 51,  0,204}, {  0,204,  0}, {102,255,255},
{255, 51,255}, {255,102,  0}, {204,  0, 51}, {153,102,204}, {153,102,  0}, {102,153, 51}, {102,204,153}, {102,102,255}, {255,153,  0}, {255, 51,  0},
{153,255, 51}, {204,102, 51}, {  0,102, 51}, {153, 51, 51}, {204,153,204}, { 51,153,153}, {204,255, 51}, {255,102, 51}, { 51,102,255}, {204,255,  0},
{ 51,  0,153}, {204,153,255}, {  0,153,153}, {153, 51,102}, {153,255,204}, { 51,  0, 51}, {255,204, 51}, {  0,204,255}, {102,204, 51}, {204,153,102},
{ 51,255,153}, {204,  0,204}, {102, 51,153}, { 51,204, 51}, {102,255, 51}, {102,204,102}, { 51,255,255}, {153,  0,153}, {204,255,255}, { 51,255, 51},
{  0,255,  0}, {  0,204,153}, {204,204,255}, { 51,204,  0}, {153,255,153}, {  0,153,102}, {204,  0,153}, {102,153,255}, { 51, 51,  0}, {102,204,  0},
{ 51,102,102}, {204, 51,  0}, {204,  0,255}, {  0,102,204}, {102,  0,204}, {  0,  0,153}, {255,  0, 51}, {204,204, 51}, {153,204,255}, {  0,255, 51},
{255,204,153}, {  0, 51,204}, {153,153,255}, {102, 51,204}, {255,153,153}, {153,  0,102}, { 51,153,  0}, { 51,102,153}, {153, 51,  0}, {102,  0,153},
{204,204,  0}, {  0, 51,255}, {204, 51,102}, {102,153,102}, {153,  0,204}, { 51, 51,102}, {102,102,204}, { 51,255,102}, {102,  0, 51}, {153,204, 51},
{255,153,102}, {  0,153,  0}, {255,204,102}, {255,102,153}, {255,  0,102}, {153,  0, 51}, {204,  0,102}, {255,204,  0}, {255, 51,204}, {204, 51,153}};

class MapGenerator
{
    public:

        std::string mapname_;
        ros::Subscriber map_sub_;
        ros::Publisher pub;
        bool saved_map_;
        int count;
    
        MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
        {
            ros::NodeHandle n;
            ROS_INFO("Waiting for the map");
            map_sub_ = n.subscribe("/spco/color_map", 1, &MapGenerator::mapCallback, this);
            pub = n.advertise<std_msgs::String>("/spco/mapsaved", 1000);
            count = 0;
        }

        void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
        {
            ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

            count++;
            std::stringstream ss;
            ss << count;
            std::string mapdatafile = mapname_ + "/" + ss.str() + ".ppm";

            ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
            FILE* out = fopen(mapdatafile.c_str(), "w");
            if (!out)
            {
                ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
                return;
            }

            fprintf(out, "P6\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n", map->info.resolution, map->info.width, map->info.height);
            for(unsigned int y = 0; y < map->info.height; y++)
            {
                for(unsigned int x = 0; x < map->info.width; x++)
                {
                    unsigned int i = x + (map->info.height - y - 1) * map->info.width;
                    if (map->data[i] >= 0)
                    {
                        fputc(COLOR[map->data[i]][0], out);
                        fputc(COLOR[map->data[i]][1], out);
                        fputc(COLOR[map->data[i]][2], out);
                    }
                    else if (map->data[i] == -2)
                    {
                        fputc(000, out);
                        fputc(000, out);
                        fputc(000, out);
                    }
                    else
                    {
                        fputc(205, out);
                        fputc(205, out);
                        fputc(205, out);
                    }
                }
            }

            fclose(out);

            ROS_INFO("Done\n");
            saved_map_ = true;

            std_msgs::String msg;
            msg.data = mapdatafile;
            pub.publish(msg);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_saver");
    ros::NodeHandle node_private("~");
    std::string hoge;
    node_private.getParam("traialname", hoge);
    std::cout << hoge << std::endl;
    std::string mapname = "PATH/catkin_ws/src/spco_mapping/data/" + hoge;

    MapGenerator mg(mapname);
    ros::spin();

    return 0;
}

