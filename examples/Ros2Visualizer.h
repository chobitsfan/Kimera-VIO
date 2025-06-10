#pragma once

#include <kimera-vio/frontend/FrontendOutputPacketBase.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
// #include "KimeraRos2Node.h"
#include "KimeraRos2NodeCommon.h"

class Ros2Visualizer : public VIO::Visualizer3D {
    public:
        KIMERA_POINTER_TYPEDEFS(Ros2Visualizer);
        KIMERA_DELETE_COPY_CONSTRUCTORS(Ros2Visualizer);

        Ros2Visualizer(std::shared_ptr<KimeraRos2NodeCommon> ros2_node);
        // Ros2Visualizer(const VIO::VioParams& vio_params, std::shared_ptr<KimeraRos2NodeCommon> ros2_node);
        virtual ~Ros2Visualizer() = default;

        /**
        * @brief spinOnce
        * Spins the display once to render the visualizer output.
        * @param viz_input
        */
        VIO::VisualizerOutput::UniquePtr spinOnce(const VIO::VisualizerInput& viz_input) override;

    private:
        std::shared_ptr<KimeraRos2NodeCommon> ros2_node_;
};
