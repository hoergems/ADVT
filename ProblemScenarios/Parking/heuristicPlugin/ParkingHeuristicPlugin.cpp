#include <oppt/plugin/Plugin.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include <oppt/opptCore/geometric/Sphere.hpp>
#include "ParkingHeuristicOptions.hpp"
#include "VehicleState.hpp"

namespace oppt
{
class ParkingHeuristicPlugin: public HeuristicPlugin
{
public:
    ParkingHeuristicPlugin():
        HeuristicPlugin() {
    }

    virtual ~ParkingHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ParkingHeuristicOptions>(optionsFile);
        parseGoalArea();
        auto options = static_cast<const ParkingHeuristicOptions *>(options_.get());
        stepPenalty = options->stepPenalty;
        exitReward = options->exitReward;
        return true;
    }

    virtual double getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        auto options = static_cast<const ParkingHeuristicOptions *>(options_.get());
        auto vehicleState = heuristicInfo->currentState->as<VehicleState>();
        VectorFloat vehiclePos({vehicleState->x(), vehicleState->y(), vehicleState->z()});
        FloatType distanceToGoal = math::euclideanDistance(vehiclePos, goalAreaPosition_);

        FloatType heuristic = 0.0;
        FloatType discountPower = std::pow(options->discountFactor , distanceToGoal);
        heuristic -= stepPenalty * (discountPower - 1) / std::log(options->discountFactor);
        heuristic += exitReward* discountPower;
        return heuristic;
    }

private:
    VectorFloat goalAreaPosition_;

    FloatType goalAreaRadius_ = 1.0;

    FloatType stepPenalty = 0.0;

    FloatType exitReward = 0.0;

private:
    gazebo::physics::Link *getLinkPtr(const RobotEnvironment *robotEnvironment, const std::string &name) {
        auto links = robotEnvironment->getGazeboInterface()->getLinks();
        gazebo::physics::Link *foundLink = nullptr;
        for (auto &link : links) {
            if (link->GetScopedName() == name) {
                foundLink = link;
                break;
            }
        }

        if (!foundLink) {
            WARNING("Link '" + name + "' could not be found");
            getchar();
        }

        return foundLink;
    }

    void parseGoalArea() {
        auto goalAreaLink = getLinkPtr(robotEnvironment_, "GoalArea::GoalAreaLink");
        if (!goalAreaLink)
            ERROR("GoalArea link could not be found");
        auto goalAreaPose = goalAreaLink->GetModel()->WorldPose();
        goalAreaPosition_ = VectorFloat({goalAreaPose.Pos().X(), goalAreaPose.Pos().Y(), goalAreaPose.Pos().Z()});

        auto bodies = robotEnvironment_->getScene()->getBodies();
        bool goalSphereFound = false;
        for (auto &body : bodies) {
            if (body->getScopedName().find("GoalArea::GoalAreaLink") != std::string::npos) {
                auto geom = body->getVisualGeometries()[0];
                goalAreaRadius_ = geom->as<geometric::Sphere>()->getRadius();
                goalSphereFound = true;
                break;
            }
        }

        if (goalSphereFound == false)
            ERROR("Goal sphere could not be found");
    }
};

OPPT_REGISTER_HEURISTIC_PLUGIN(ParkingHeuristicPlugin)

}