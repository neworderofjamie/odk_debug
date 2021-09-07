#include <cstdlib>

// BoB robotics includes
#include "common/main.h"
#include "imgproc/roll.h"
#include "navigation/perfect_memory.h"
#include "viz/plot_ridf.h"

// Third party includes
#include "third_party/csv.h"
#include "third_party/path.h"
#include "plog/Log.h"

// Bounds used for extracting masks from ODK2 images
const cv::Scalar odk2MaskLowerBound(1, 1, 1);
const cv::Scalar odk2MaskUpperBound(255, 255, 255);

using namespace units::angle;
using namespace units::literals;

int bobMain(int argc, char **argv)
{
    BoBRobotics::Navigation::PerfectMemoryRotater<> pm(cv::Size(256, 63));
    const filesystem::path dataPath{"office"};


    // Load snapshot
    const cv::Mat trainSnapshot = cv::imread((dataPath / "snapshot_0.png").str(), cv::IMREAD_GRAYSCALE);
    assert(!trainSnapshot.empty());

    // Build mask
    const BoBRobotics::ImgProc::Mask trainMask(trainSnapshot, odk2MaskLowerBound, odk2MaskUpperBound);

    // Add to PM
    pm.train(trainSnapshot, trainMask);
     
    const cv::Mat testSnapshot = cv::imread((dataPath / "snapshot_287.png").str(), cv::IMREAD_GRAYSCALE);
    assert(!testSnapshot.empty());

    // Build mask
    const BoBRobotics::ImgProc::Mask testMask(testSnapshot, odk2MaskLowerBound, odk2MaskUpperBound);

    const auto &differences = pm.getImageDifferences(testSnapshot, testMask);
    std::vector<float> differenceVector;
    for(size_t i = 0; i < differences.row(0).size(); i++) {
        differenceVector.push_back(differences.row(0)(i));
        std::cout << differenceVector.back() << "," << std::endl;
    }
    const auto lowestDifference = std::distance(differenceVector.cbegin(), std::min_element(differenceVector.cbegin(), differenceVector.cend()));
    cv::Mat rotatedTest;
    BoBRobotics::ImgProc::roll(testSnapshot, rotatedTest, lowestDifference);

    cv::Mat diffImage;
    cv::absdiff(rotatedTest, trainSnapshot, diffImage);
    applyColorMap(diffImage, diffImage, cv::COLORMAP_JET);

    // Get best heading from left side of scan
    /*degree_t bestHeading;
    float lowestDifference;
    size_t bestSnapshot;
    std::tie(bestHeading, bestSnapshot, lowestDifference, std::ignore) = pm.getHeading(testSnapshot, testMask);
    LOGI << bestHeading.value() << " deg ," << lowestDifference << "," << bestSnapshot;*/

    cv::namedWindow("train", cv::WINDOW_NORMAL);
    cv::namedWindow("train mask", cv::WINDOW_NORMAL);
    cv::namedWindow("test", cv::WINDOW_NORMAL);
    cv::namedWindow("test rotated", cv::WINDOW_NORMAL);
    cv::namedWindow("difference", cv::WINDOW_NORMAL);
    cv::namedWindow("test mask", cv::WINDOW_NORMAL);
    
    cv::imshow("train", trainSnapshot);
    cv::imshow("train mask", trainMask.get());
    cv::imshow("test", testSnapshot);
    cv::imshow("test rotated", rotatedTest);
    cv::imshow("difference", diffImage);
    cv::imshow("test mask", testMask.get());
    
    do
    {
    } while(cv::waitKey(1) != 27);
    /*{
        // Open Training CSV
        io::CSVReader<1> trainingCSV((dataPath / "training.csv").str());
        trainingCSV.read_header(io::ignore_extra_column, "Filename");

        // Read snapshot filenames from file
        std::string snapshotFile;
        while(trainingCSV.read_row(snapshotFile)) {
            LOGI << "Training on" << snapshotFile;

            // Load snapshot
            const cv::Mat snapshot = cv::imread(snapshotFile, cv::IMREAD_GRAYSCALE);
            assert(!snapshot.empty());

            // Build mask
            const BoBRobotics::ImgProc::Mask mask(snapshot, odk2MaskLowerBound, odk2MaskUpperBound);

            // Add to PM
            pm.train(snapshot, mask);
        }
    }

    {
        const size_t numScanColumns = (size_t)std::round(turn_t(90_deg).value() * 256.0);

        // Open Testing CSV
        io::CSVReader<4> testingCSV((dataPath / "testing.csv").str());
        testingCSV.read_header(io::ignore_extra_column, "Best heading [degrees]", "Lowest difference", "Best snapshot index", "Filename");

        // Read test points from file
        double bestHeading;
        double lowestDifference;
        unsigned int bestSnapshotIndex;
        std::string snapshotFile;
        while(testingCSV.read_row(bestHeading, lowestDifference, bestSnapshotIndex, snapshotFile)) {
             // Load snapshot
            const cv::Mat snapshot = cv::imread((dataPath / snapshotFile).str(), cv::IMREAD_GRAYSCALE);
            assert(!snapshot.empty());

            // Build mask
            const BoBRobotics::ImgProc::Mask mask(snapshot, odk2MaskLowerBound, odk2MaskUpperBound);

            // Get best heading from left side of scan
            degree_t leftBestHeading;
            float leftLowestDifference;
            size_t leftBestSnapshot;
            std::tie(leftBestHeading, leftBestSnapshot, leftLowestDifference, std::ignore) = pm.getHeading(
                snapshot, mask, 1, 0, numScanColumns);

            // Get best heading from right side of scan
            degree_t rightBestHeading;
            float rightLowestDifference;
            size_t rightBestSnapshot;
            std::tie(rightBestHeading, rightBestSnapshot, rightLowestDifference, std::ignore) = pm.getHeading(
                snapshot, mask, 1, 256 - numScanColumns, 256);

            // If best result came from left scan
        if(leftLowestDifference < rightLowestDifference) {
            LOGI << "Lowest difference: " << (leftLowestDifference / 255.0f) << "(" << lowestDifference << "), Best heading:" << leftBestHeading << "(" << bestHeading << "), Best snapshot: " << leftBestSnapshot << "(" << bestSnapshotIndex << ")";
        }
        else {
            LOGI << "Lowest difference: " << (rightLowestDifference / 255.0f) << "(" << lowestDifference << "), Best heading:" << rightBestHeading << "(" << bestHeading << "), Best snapshot: " << rightBestSnapshot << "(" << bestSnapshotIndex << ")";
        }

        }
    }*/

    return EXIT_SUCCESS;
}
