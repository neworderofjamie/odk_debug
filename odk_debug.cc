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

    {
        // Open Training CSV
        io::CSVReader<1> trainingCSV((dataPath / "training.csv").str());
        trainingCSV.read_header(io::ignore_extra_column, "Filename");

        // Read snapshot filenames from file
        std::string snapshotFile;
        while(trainingCSV.read_row(snapshotFile)) {
            LOGI << "Training on" << snapshotFile;

            // Load snapshot
            cv::Mat snapshot = cv::imread(snapshotFile);
            assert(!snapshot.empty());

            // Build mask
            const BoBRobotics::ImgProc::Mask mask(snapshot, odk2MaskLowerBound, odk2MaskUpperBound);

            // Convert to grayscale
            cv::cvtColor(snapshot, snapshot, cv::COLOR_BGR2GRAY);

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
            cv::Mat snapshot = cv::imread((dataPath / snapshotFile).str());
            assert(!snapshot.empty());

            // Build mask
            const BoBRobotics::ImgProc::Mask mask(snapshot, odk2MaskLowerBound, odk2MaskUpperBound);

            // Convert to grayscale
            cv::cvtColor(snapshot, snapshot, cv::COLOR_BGR2GRAY);

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
                LOGI << "Lowest difference: " << (leftLowestDifference / 255.0f) << "(" << lowestDifference << "), Best heading:" << leftBestHeading.value() << "(" << bestHeading << "), Best snapshot: " << leftBestSnapshot << "(" << bestSnapshotIndex << ")";
            }
            else {
                LOGI << "Lowest difference: " << (rightLowestDifference / 255.0f) << "(" << lowestDifference << "), Best heading:" << rightBestHeading.value() << "(" << bestHeading << "), Best snapshot: " << rightBestSnapshot << "(" << bestSnapshotIndex << ")";
            }

        }
    }

    return EXIT_SUCCESS;
}
