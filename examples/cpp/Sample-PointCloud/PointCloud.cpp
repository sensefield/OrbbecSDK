#include "libobsensor/ObSensor.hpp"
// #include "opencv2/opencv.hpp"
#include <fstream>
#include <iostream>
#include "utils.hpp"

#define KEY_ESC 27

void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    OBPoint *point = (OBPoint *)frame->data();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

int main(int argc, char **argv) try {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    ob::Pipeline pipeline;
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    std::shared_ptr<ob::StreamProfileList> depthProfileList;
    depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    if(depthProfileList->count() > 0) {
        std::shared_ptr<ob::StreamProfile> depthProfile;
        depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
        config->enableStream(depthProfile);
    }

    pipeline.start(config);
    ob::PointCloudFilter pointCloud;
    auto cameraParam = pipeline.getCameraParam();
    pointCloud.setCameraParam(cameraParam);

    int count = 0;
    while(true) {
        auto frameset = pipeline.waitForFrames(100);
        if(kbhit()) {
            int key = getch();
            if(key == KEY_ESC) {
                break;
            }            
            else if(key == 'D' || key == 'd') {
                count = 0;
                // Limit up to 10 repetitions
                while(count++ < 10) {
                    // Wait for up to 100ms for a frameset in blocking mode.
                    auto frameset = pipeline.waitForFrames(100);
                    if(frameset != nullptr && frameset->depthFrame() != nullptr) {
                        // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                        // millimeter)
                        auto depthValueScale = frameset->depthFrame()->getValueScale();
                        pointCloud.setPositionDataScaled(depthValueScale);
                        try {
                            // generate point cloud and save
                            std::cout << "Save Depth PointCloud to ply file..." << std::endl;
                            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
                            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                            savePointsToPly(frame, "DepthPoints.ply");
                            std::cout << "DepthPoints.ply Saved" << std::endl;
                        }
                        catch(std::exception &e) {
                            std::cout << "Get point cloud failed" << std::endl;
                        }
                        break;
                    }
                }
            }
        }
    }
    // stop the pipeline
    pipeline.stop();
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
