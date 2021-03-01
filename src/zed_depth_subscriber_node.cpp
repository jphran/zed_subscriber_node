//
// Created by jfrancis on 2/26/21.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

struct Point {
    int x;
    int y;
    float depth;
    explicit Point(int xLoc = 0, int yLoc = 0, float ptDepth = std::nanf("0"))
        : x(xLoc)
        , y(yLoc)
        , depth(ptDepth){};
    ~Point() = default;;
};
struct BoundingBox {
    Point upperLeft;
    Point lowerRight;
    float depth;
    bool isTracking;
    int imgWidth;
    int imgHeight;
    BoundingBox()
        : upperLeft(Point())
        , lowerRight(Point())
        , depth(0.0f)
        , isTracking(false)
        , imgWidth(0)
        , imgHeight(0){};
};

union ImageDataConverter {
    float f;
    uint8_t byte[4];
} ImageData;

void findNearest(BoundingBox& bb, std::vector<float> depths, int kernelSize = 4) {
    int centerX = bb.upperLeft.x + ((bb.lowerRight.x - bb.upperLeft.x) / 2);
    int centerY = bb.upperLeft.y + ((bb.lowerRight.y - bb.upperLeft.y) / 2);
    int centerIdx = centerX + bb.imgWidth * centerY;

//    ROS_INFO("center pixel, dist: %d, %g m", centerIdx, depths[centerIdx]);
    int nearestPixelIdx = centerIdx;
    int nearestPixelX = centerX;
    int nearestPixelY = centerY;

    // find the closest pixel in the bounding box
    for(int i = bb.upperLeft.y; i < bb.lowerRight.y; i += kernelSize) {
        for(int j = bb.upperLeft.x; j < bb.lowerRight.x; j += kernelSize) {
            int nextPixelIdx = j + bb.imgWidth * i;
            if(isnormal(depths[nextPixelIdx])
                && depths[nextPixelIdx] < depths[nearestPixelIdx]) {
                nearestPixelIdx = nextPixelIdx;
                nearestPixelX = j;
                nearestPixelY = i;
            }
        }
    }

//    if(nearestPixelIdx)
    ROS_INFO("nearest pixel idx, depth: %d, %g m", nearestPixelIdx, depths[nearestPixelIdx]);

    // if the pixel depth is valid, set the bounding box
    if(isnormal(depths[nearestPixelIdx]) && not isinf(depths[nearestPixelIdx])) {
        int dX = bb.imgWidth / 10;
        int dY = bb.imgHeight / 10;
        bb.upperLeft.x = nearestPixelX - dX;
        bb.upperLeft.y = nearestPixelY - dY;
        bb.lowerRight.x = nearestPixelX + dX;
        bb.lowerRight.y = nearestPixelY + dY;

        bb.depth = depths[nearestPixelIdx];
        bb.isTracking = true;
        ROS_INFO_THROTTLE(0.5, "good track, bb: (%d, %d), (%d, %d)", bb.upperLeft.x, bb.upperLeft.y, bb.lowerRight.x, bb.lowerRight.y);
    }
    else {
        bb.isTracking = false;
        bb.upperLeft = Point(0, 0);
        bb.lowerRight = Point(bb.imgWidth, bb.imgHeight);
        ROS_INFO("lost track");
    }



}

/**
 * Subscriber callback
 */
void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    BoundingBox bb;
//     Get a pointer to the depth values casting the data
//     pointer to floating point
    float depthsArr[msg->data.size() / 4];
    std::memcpy(depthsArr, &msg->data[0], sizeof(depthsArr));
    std::vector<float> depths(depthsArr, depthsArr + sizeof(depthsArr) / sizeof(depthsArr[0]));
//    auto depths = (float*) &msg->data[0];

    // Image coordinates of the center pixel
    int u = msg->width / 2;
    int v = msg->height / 2;

    // Linear index of the center pixel
    int centerIdx = u + msg->width * v;

    // Output the measure
//    ROS_INFO("Center pixel, distance : %d, %g m", centerIdx, depths[centerIdx]);


    bb.imgWidth = msg->width;
    bb.imgHeight = msg->height;
    bb.upperLeft = Point(u - 100, v - 20);
    bb.lowerRight = Point(u + 100, v + 20);
    findNearest(bb, depths);
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "zed_depth_subscriber");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called imageCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber subDepth = n.subscribe("/zed2/zed_node/depth/depth_registered", 10, depthCallback);


    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}